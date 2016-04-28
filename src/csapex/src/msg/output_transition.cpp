/// HEADER
#include <csapex/msg/output_transition.h>

/// COMPONENT
#include <csapex/model/node_handle.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/input.h>

using namespace csapex;

OutputTransition::OutputTransition(delegate::Delegate0<> activation_fn)
    : Transition(activation_fn), sequence_number_(-1)
{
}
OutputTransition::OutputTransition()
    : Transition(), sequence_number_(-1)
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
    auto ca = output->connection_added.connect([this](ConnectionPtr connection) {
            addConnection(connection);
});
    output_signal_connections_[output].push_back(ca);

    auto cf = output->connection_faded.connect([this](ConnectionPtr connection) {
            removeConnection(connection);
});
    output_signal_connections_[output].push_back(cf);

    auto cp = output->message_processed.connect([this]() {
        publishNextMessage();
    });
    output_signal_connections_[output].push_back(cp);
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

bool OutputTransition::isSink() const
{
    for(auto pair : outputs_) {
        OutputPtr output = pair.second;
        if(output->isConnected()) {
            return false;
        }
    }
    return true;
}

void OutputTransition::sendMessages()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    updateConnections();

    //        std::cerr << "commit messages output transition: " << node_->getUUID() << std::endl;

    for(auto pair : outputs_) {
        OutputPtr output = pair.second;
//        if(output->isConnected()) {
            output->commitMessages();
//        }
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

    if(isSink()) {
        setOutputsIdle();
    }
}

void OutputTransition::publishNextMessage()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    if(!areAllConnections(Connection::State::DONE)) {
        return;
    }

    apex_assert_hard(areAllConnections(Connection::State::DONE));

    for(auto pair : outputs_) {
        OutputPtr output = pair.second;
        output->nextMessage();
    }
    if(areOutputsIdle()) {
        if(areAllConnections(Connection::State::DONE)) {
            updateConnections();

            messages_processed();
        }

    } else {
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

    apex_assert_hard(areAllConnections(Connection::State::DONE));

    for(auto pair : outputs_) {
        OutputPtr out = pair.second;
        apex_assert_hard(out);

        out->publish();
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
