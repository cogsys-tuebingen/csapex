/// HEADER
#include <csapex/msg/output_transition.h>

/// COMPONENT
#include <csapex/model/node_handle.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/input.h>
#include <csapex/msg/no_message.h>
#include <csapex/utility/debug.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

OutputTransition::OutputTransition(delegate::Delegate0<> activation_fn)
    : Transition(activation_fn), sequence_number_(-1), try_to_publish_(false)
{
}
OutputTransition::OutputTransition()
    : Transition(), sequence_number_(-1), try_to_publish_(false)
{
}

void OutputTransition::reset()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(auto pair : outputs_) {
        OutputPtr output = pair.second;
        output->reset();
    }
    for(ConnectionPtr connection : connections_) {
        connection->reset();
    }

    Transition::reset();
}

std::vector<UUID> OutputTransition::getOutputs() const
{
    std::vector<UUID> res;
    for(const auto& pair : outputs_) {
        res.push_back(pair.second->getUUID());
    }
    std::sort(res.begin(), res.end());
    return res;
}

OutputPtr OutputTransition::getOutput(const UUID& id) const
{
    return outputs_.at(id);
}

void OutputTransition::addOutput(OutputPtr output)
{    
    output->setOutputTransition(this);

    output->setSequenceNumber(sequence_number_);

    // remember the output
    outputs_[output->getUUID()] = output;

    // connect signals
    auto ca = output->connection_added.connect([this](const ConnectionPtr& connection) {
        bool needs_message = !areOutputsIdle();

        addConnection(connection);

        if(needs_message) {
            connection->setToken(Token::makeEmpty<connection_types::NoMessage>());
        }
    });
    output_signal_connections_[output].push_back(ca);

    auto cf = output->connection_faded.connect([this](const ConnectionPtr& connection) {
        removeConnection(connection);
    });
    output_signal_connections_[output].push_back(cf);

    auto cp = output->message_processed.connect([this](Connectable*) {
        publishNextMessage();
    });
    output_signal_connections_[output].push_back(cp);

//    auto cr = output->connection_removed_to.connect([this](Connectable* output) {
//        if(output->isEnabled()) {
//            publishNextMessage();
//        }
//    });
//    output_signal_connections_[output].push_back(cr);
}

void OutputTransition::removeOutput(OutputPtr output)
{
    output->removeOutputTransition();

    // disconnect signals
    for(auto f : output_signal_connections_[output]) {
        f.disconnect();
    }
    output_signal_connections_.erase(output);

    // forget the output
    outputs_.erase(output->getUUID());
}

void OutputTransition::setSequenceNumber(long seq_no)
{
    sequence_number_ = seq_no;

    for(auto pair : outputs_) {
        OutputPtr output = pair.second;
        output->setSequenceNumber(sequence_number_);
    }
}

long OutputTransition::getSequenceNumber() const
{
    return sequence_number_;
}

bool OutputTransition::isEnabled() const
{
    return canStartSendingMessages();
}

void OutputTransition::connectionRemoved(Connection *connection)
{
    Transition::connectionRemoved(connection);

    if(connections_.empty()) {
        //        node_->notifyMessagesProcessed();
    }
}

void OutputTransition::connectionAdded(Connection *connection)
{
    Transition::connectionAdded(connection);

    if(isEnabled()) {
        updateConnections();
    }
}

bool OutputTransition::canStartSendingMessages() const
{
    for(auto pair : outputs_) {
        OutputPtr output = pair.second;
        if(output->isEnabled() && output->isConnected()) {
            if(output->getState() != Output::State::IDLE) {
                return false;
            }
        }
    }
    return areAllConnections(Connection::State::DONE, Connection::State::NOT_INITIALIZED);
}

bool OutputTransition::sendMessages(bool is_active)
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    apex_assert_hard(areAllConnections(Connection::State::NOT_INITIALIZED));

    updateConnections();

    bool has_sent_activator_message = false;

    //        std::cerr << "commit messages output transition: " << node_->getUUID() << std::endl;

    for(auto pair : outputs_) {
        OutputPtr output = pair.second;
        has_sent_activator_message |= output->commitMessages(is_active);
    }

    long seq_no = -1;
    for(auto pair : outputs_) {
        OutputPtr output = pair.second;
        long s = output->sequenceNumber();
        if(seq_no == -1) {
            seq_no = s;
        } else {
            apex_assert_hard(seq_no == s);
        }
    }

    setSequenceNumber(seq_no);

    fillConnections();

    if(!hasConnection()) {
        setOutputsIdle();
    }

    return has_sent_activator_message;
}

void OutputTransition::publishNextMessage()
{
    std::unique_lock<std::recursive_mutex> lock(sync, std::try_to_lock);
    if(!lock) {
        try_to_publish_ = true;
        return;
    }


    if(!areAllConnections(Connection::State::DONE)) {
        APEX_DEBUG_CERR <<"cannot publish next, not all connections are done" << std::endl;
        return;
    }

    apex_assert_hard(areAllConnections(Connection::State::DONE));

    for(auto pair : outputs_) {
        OutputPtr output = pair.second;
        output->nextMessage();
    }
    if(areOutputsIdle()) {
        APEX_DEBUG_CERR <<"all outputs are idle" << std::endl;
        if(areAllConnections(Connection::State::DONE)) {
            APEX_DEBUG_CERR <<"all outputs are done" << std::endl;
            updateConnections();

            messages_processed();
        }

    } else {
        APEX_DEBUG_CERR <<"some outputs are not idle" << std::endl;
        fillConnections();
    }
}

bool OutputTransition::areOutputsIdle() const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(auto pair : outputs_) {
        OutputPtr output = pair.second;
        if(output->getState() != Output::State::IDLE) {
            return false;
        }
    }
    return true;
}

void OutputTransition::fillConnections()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    apex_assert_hard(outputs_.empty() || !areOutputsIdle());

    apex_assert_hard(areAllConnections(Connection::State::NOT_INITIALIZED));

    for(auto pair : outputs_) {
        OutputPtr out = pair.second;
        apex_assert_hard(out);

        out->publish();
    }

    if(try_to_publish_) {
        publishNextMessage();
    }
}

void OutputTransition::clearBuffer()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(auto pair : outputs_) {
        OutputPtr output = pair.second;
        output->clearBuffer();
    }
}

void OutputTransition::setOutputsIdle()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(auto pair : outputs_) {
        OutputPtr output = pair.second;
        output->setState(Output::State::IDLE);
    }
}
