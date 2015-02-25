/// HEADER
#include <csapex/msg/output_transition.h>

/// COMPONENT
#include <csapex/model/node_worker.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>
#include <csapex/utility/assert.h>

using namespace csapex;

OutputTransition::OutputTransition(NodeWorker *node)
    : Transition(node)
{

}
bool OutputTransition::canSendMessages() const
{
    for(Output* output : node_->getMessageOutputs()) {
        if(output->isEnabled() && output->isConnected() && output->getState() != Output::State::IDLE) {
            return false;
        }
    }
    return true;
}

bool OutputTransition::isSink() const
{
    for(Output* output : node_->getMessageOutputs()) {
        if(output->isConnected()) {
            return false;
        }
    }
    return true;
}

void OutputTransition::sendMessages()
{
    apex_assert_hard(!isSink());
    std::cerr << "commit messages output transition: " << node_->getUUID() << std::endl;
    for(Output* out : node_->getMessageOutputs()) {
        if(out->isConnected()) {
            out->commitMessages();
        }
    }

    apex_assert_hard(areConnections(Connection::State::READY_TO_RECEIVE));

    std::cerr << "fill first time: " << node_->getUUID() << std::endl;
    fillConnections();
}

void OutputTransition::notifyMessageProcessed()
{
    if(!areConnections(Connection::State::READ)) {
        return;
    }

    for(Output* out : node_->getMessageOutputs()) {
        out->nextMessage();
    }
    if(areOutputsDone()) {
        if(areConnections(Connection::State::READ)) {
            if(node_->getState() == NodeWorker::State::WAITING_FOR_OUTPUTS) {
                std::cerr << "all outputs are done: " << node_->getUUID() << std::endl;
                node_->notifyMessagesProcessed();
            }
        }

    } else {
        std::cerr << "fill again: " << node_->getUUID() << std::endl;
        fillConnections();
    }
}

bool OutputTransition::areOutputsDone() const
{
    for(Output* out : node_->getMessageOutputs()) {
        if(out->getState() != Output::State::IDLE) {
            return false;
        }
    }
    return true;
}

void OutputTransition::fillConnections()
{
    std::cerr << "fill connections output transition: " << node_->getUUID() << std::endl;
    for(ConnectionWeakPtr c : connections_) {
        ConnectionPtr connection = c.lock();
        if(connection->isEnabled()) {
            Output* out = dynamic_cast<Output*>(connection->from());
            apex_assert_hard(out);
            auto msg = out->getMessage();
            apex_assert_hard(msg);
            connection->setMessage(msg);
            apex_assert_hard(connection->getState() == Connection::State::UNREAD);
        }
    }
    for(ConnectionWeakPtr c : connections_) {
        ConnectionPtr connection = c.lock();
        if(connection->isEnabled()) {
            apex_assert_hard(connection->getState() == Connection::State::UNREAD);
            connection->notifyMessageSet();
        }
    }
}

void OutputTransition::setConnectionsReadyToReceive()
{
    apex_assert_hard(node_->getState() == NodeWorker::State::PROCESSING);
    apex_assert_hard(areConnections(Connection::State::READ, Connection::State::NOT_INITIALIZED));

    for(ConnectionWeakPtr c : connections_) {
        c.lock()->setState(Connection::State::READY_TO_RECEIVE);
    }
    for(Output* output : node_->getMessageOutputs()) {
        output->clear();
    }
}
