/// HEADER
#include <csapex/msg/output_transition.h>

/// COMPONENT
#include <csapex/model/node_worker.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/input.h>

using namespace csapex;

OutputTransition::OutputTransition(NodeWorker *node)
    : Transition(node), outputs_done_(true)
{

}

void OutputTransition::reset()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(Output* output : node_->getAllOutputs()) {
        output->reset();
    }
    for(ConnectionPtr connection : established_connections_) {
        connection->reset();
    }
    outputs_done_ = true;
}


void OutputTransition::connectionAdded(Connection *connection)
{
    connection->endpoint_established.connect([this]() {
        // establish();
        node_->triggerCheckTransitions();
    });

    //    if(node_->getState() == NodeWorker::State::IDLE || node_->getState() == NodeWorker::State::ENABLED) {
    //        establish();
    //    }
}

void OutputTransition::connectionRemoved(Connection *connection)
{
    connection->fadeSource();

    if(established_connections_.empty()) {
        outputs_done_ = true;
//        node_->notifyMessagesProcessed();
    }
}

void OutputTransition::establish()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    auto unestablished_connections = unestablished_connections_;
    lock.unlock();

    if(!unestablished_connections.empty()) {
        for(auto c : unestablished_connections) {
            if(!c->isSourceEstablished()) {
                c->establishSource();
            }
            if(c->isSourceEstablished() && c->isSinkEstablished()) {
                establishConnection(c);
            }
        }
    }
}

bool OutputTransition::canSendMessages() const
{
    if(!outputs_done_) {
        return false;
    }
    for(Output* output : node_->getAllOutputs()) {
        if(output->isEnabled() && output->isConnected()) {
            if(output->getState() == Output::State::ACTIVE) {
                return false;
            }
        }
    }
    return true;
}

bool OutputTransition::isSink() const
{
    for(Output* output : node_->getAllOutputs()) {
        if(output->isConnected()) {
            return false;
        }
    }
    return true;
}

void OutputTransition::sendMessages()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    int non_forced_connections = 0;

    apex_assert_hard(!isSink());
    //        std::cerr << "commit messages output transition: " << node_->getUUID() << std::endl;

    bool has_multipart = false;
    bool multipart_are_done = true;

    for(Input* in : node_->getAllInputs()) {
        for(auto& c : in->getConnections()) {
            int f = c->getMessage()->flags.data;
            if(f & (int) ConnectionType::Flags::Fields::MULTI_PART) {
                has_multipart = true;

                bool last_part = f & (int) ConnectionType::Flags::Fields::LAST_PART;
                multipart_are_done &= last_part;
            }
        }
    }

    for(Output* out : node_->getAllOutputs()) {
        if(out->isConnected()) {
            out->commitMessages();

            if(!out->isForced()) {
                ++non_forced_connections;
            }
        }
    }


    if(has_multipart) {
        for(ConnectionPtr connection : established_connections_) {
            Output* out = dynamic_cast<Output*>(connection->from());
            if(out) {
                bool is_last = multipart_are_done;
                out->setMultipart(true, is_last);
            }
        }
    }

    outputs_done_ = false;

    if(non_forced_connections == 0) {
        // all outputs are forced -> there is no connections that can send back a notification!

        apex_assert_hard(areConnections(Connection::State::DONE));
        updateOutputs();

    } else {
        // at least one output is not forced and will send back a notification
//        apex_assert_hard(areConnections(Connection::State::READY_TO_RECEIVE));

        //    std::cerr << "fill first time: " << node_->getUUID() << std::endl;
        fillConnections();
    }

}

void OutputTransition::updateOutputs()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    if(!areConnections(Connection::State::DONE)) {
        return;
    }

    apex_assert_hard(areConnections(Connection::State::DONE));

    if(outputs_done_) {
        //        std::cerr << "supressing notifyMessageProcessed in output" << std::endl;
        return;
    }

    apex_assert_hard(node_->getState() == NodeWorker::State::WAITING_FOR_OUTPUTS);

    for(Output* out : node_->getAllOutputs()) {
        out->nextMessage();
    }
    if(areOutputsIdle()) {
        if(areConnections(Connection::State::DONE)) {
            //            std::cerr << "all outputs are done: " << node_->getUUID() << std::endl;
            outputs_done_ = true;
            //            establish();


            if(hasFadingConnection()) {
                removeFadingConnections();
            }

            node_->notifyMessagesProcessed();
        }

    } else {
        //        std::cerr << "fill again: " << node_->getUUID() << std::endl;
        //        apex_assert_hard(areConnections(Connection::State::READ));
        //        for(ConnectionPtr c : connections_) {
        //            c.lock()->setState(Connection::State::READY_TO_RECEIVE);
        //        }
        setConnectionsReadyToReceive();
        fillConnections();
    }
}

bool OutputTransition::areOutputsIdle() const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(Output* out : node_->getAllOutputs()) {
        if(out->getState() != Output::State::IDLE) {
            return false;
        }
    }
    return true;
}

void OutputTransition::fillConnections()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    apex_assert_hard(!areOutputsIdle());
//    apex_assert_hard(areConnections(Connection::State::READY_TO_RECEIVE));

    for(ConnectionPtr connection : established_connections_) {
        if(connection->isSinkEnabled()) {
            Output* out = dynamic_cast<Output*>(connection->from());
            apex_assert_hard(out);

            auto msg = out->getMessage();
            apex_assert_hard(msg);
            connection->setMessage(msg);
        }
    }

    for(ConnectionPtr connection : established_connections_) {
        if(connection->isSinkEnabled()) {
            //            apex_assert_hard(connection->getState() == Connection::State::UNREAD);
            connection->notifyMessageSet();
        }
    }
}

void OutputTransition::clearOutputs()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(Output* output : node_->getAllOutputs()) {
        output->clear();
    }
}

void OutputTransition::setConnectionsReadyToReceive()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    apex_assert_hard(areConnections(Connection::State::DONE, Connection::State::NOT_INITIALIZED));

    for(ConnectionPtr connection : established_connections_) {
        if(connection->isSinkEnabled()) {
            connection->setState(Connection::State::READY_TO_RECEIVE);
        }
    }
}
