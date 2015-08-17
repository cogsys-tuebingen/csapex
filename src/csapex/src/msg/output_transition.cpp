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
    : Transition(node)
{

}

void OutputTransition::reset()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(auto output : node_->getAllOutputs()) {
        output->reset();
    }
    for(ConnectionPtr connection : established_connections_) {
        connection->reset();
    }
}


void OutputTransition::connectionAdded(Connection *connection)
{
    Transition::connectionAdded(connection);

    trackConnection(connection, connection->endpoint_established.connect([this]() {
        // establish();
        node_->triggerCheckTransitions();
    }));

    //    if(node_->getState() == NodeWorker::State::IDLE || node_->getState() == NodeWorker::State::ENABLED) {
    //        establish();
    //    }
}

void OutputTransition::connectionRemoved(Connection *connection)
{
    Transition::connectionRemoved(connection);

    connection->fadeSource();

    if(established_connections_.empty()) {
//        node_->notifyMessagesProcessed();
    }
}

void OutputTransition::establishConnections()
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

bool OutputTransition::canStartSendingMessages() const
{
    for(auto output : node_->getAllOutputs()) {
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
    for(auto output : node_->getAllOutputs()) {
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

    for(auto out : node_->getAllOutputs()) {
        if(out->isConnected()) {
            out->commitMessages();

            if(!out->isForced()) {
                ++non_forced_connections;
            }
        }
    }

    if(non_forced_connections == 0) {
        // all outputs are forced -> there is no connections that can send back a notification!

        apex_assert_hard(areAllConnections(Connection::State::DONE));
        publishNextMessage();

    } else {
        // at least one output is not forced and will send back a notification
//        apex_assert_hard(areConnections(Connection::State::READY_TO_RECEIVE));

        //    std::cerr << "fill first time: " << node_->getUUID() << std::endl;
        fillConnections();
    }

}

void OutputTransition::publishNextMessage()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    if(!areAllConnections(Connection::State::DONE)) {
        return;
    }

    apex_assert_hard(areAllConnections(Connection::State::DONE));

    for(auto out : node_->getAllOutputs()) {
        out->nextMessage();
    }
    if(areOutputsIdle()) {
        if(areAllConnections(Connection::State::DONE)) {
            if(hasFadingConnection()) {
                removeFadingConnections();
            }

            node_->notifyMessagesProcessed();
        }

    } else {
        setConnectionsReadyToReceive();
        fillConnections();
    }
}

bool OutputTransition::areOutputsIdle() const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(auto out : node_->getAllOutputs()) {
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
            connection->notifyMessageSet();
        }
    }
}

void OutputTransition::clearOutputs()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(auto output : node_->getAllOutputs()) {
        output->clear();
    }
}

void OutputTransition::abortSendingMessages()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(auto output : node_->getAllOutputs()) {
        apex_assert_hard(output->getState() == Output::State::RECEIVING);
        output->setState(Output::State::IDLE);
    }
}

void OutputTransition::setConnectionsReadyToReceive()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    apex_assert_hard(areAllConnections(Connection::State::DONE, Connection::State::NOT_INITIALIZED));

    for(ConnectionPtr connection : established_connections_) {
        if(connection->isSinkEnabled()) {
            connection->setState(Connection::State::READY_TO_RECEIVE);
        }
    }
}
