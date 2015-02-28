/// HEADER
#include <csapex/msg/output_transition.h>

/// COMPONENT
#include <csapex/model/node_worker.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>
#include <csapex/utility/assert.h>

using namespace csapex;

OutputTransition::OutputTransition(NodeWorker *node)
    : Transition(node), notify_called_(true)
{

}
bool OutputTransition::canSendMessages() const
{
    if(!notify_called_) {
        return false;
    }
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
    std::unique_lock<std::recursive_mutex> lock(sync);

    apex_assert_hard(!isSink());
    std::cerr << "commit messages output transition: " << node_->getUUID() << std::endl;
    for(Output* out : node_->getMessageOutputs()) {
        if(out->isConnected()) {
            out->commitMessages();
        }
    }

    apex_assert_hard(areConnections(Connection::State::READY_TO_RECEIVE));

    notify_called_ = false;

    std::cerr << "fill first time: " << node_->getUUID() << std::endl;
    fillConnections();
}

void OutputTransition::notifyMessageProcessed()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    if(!areConnections(Connection::State::DONE)) {
        std::cerr << node_->getUUID() << ": cannot continue: connections are: \n";
        for(auto cw : connections_) {
            ConnectionPtr c = cw.lock();
            std::cerr << c->from()->getUUID() << " => " << c->to()->getUUID();
            std::cerr << ": " << (int) c->getState() << '\n';
        }
        std::cerr.flush();
        return;
    }

    if(notify_called_) {
        std::cerr << "supressing notifyMessageProcessed in output" << std::endl;
        return;
    }

    apex_assert_hard(node_->getState() == NodeWorker::State::WAITING_FOR_OUTPUTS);

    for(Output* out : node_->getMessageOutputs()) {
        out->nextMessage();
    }
    if(areOutputsIdle()) {
        if(areConnections(Connection::State::DONE)) {
            std::cerr << "all outputs are done: " << node_->getUUID() << std::endl;
            notify_called_ = true;
            node_->notifyMessagesProcessed();
        }

    } else {
        std::cerr << "fill again: " << node_->getUUID() << std::endl;
//        apex_assert_hard(areConnections(Connection::State::READ));
//        for(ConnectionWeakPtr c : connections_) {
//            c.lock()->setState(Connection::State::READY_TO_RECEIVE);
//        }
        setConnectionsReadyToReceive();
        fillConnections();
    }
}

bool OutputTransition::areOutputsIdle() const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(Output* out : node_->getMessageOutputs()) {
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
    apex_assert_hard(areConnections(Connection::State::READY_TO_RECEIVE));

    std::cerr << "fill connections output transition: " << node_->getUUID() << std::endl;
    for(ConnectionWeakPtr c : connections_) {
        ConnectionPtr connection = c.lock();
        if(connection->isEnabled()) {
            Output* out = dynamic_cast<Output*>(connection->from());
            apex_assert_hard(out);
            auto msg = out->getMessage();
            apex_assert_hard(msg);
            connection->setMessage(msg);
        }
    }

    for(ConnectionWeakPtr c : connections_) {
        ConnectionPtr connection = c.lock();
        if(connection->isEnabled()) {
//            apex_assert_hard(connection->getState() == Connection::State::UNREAD);
            connection->notifyMessageSet();
        }
    }
}

void OutputTransition::clearOutputs()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(Output* output : node_->getMessageOutputs()) {
        output->clear();
    }
}

void OutputTransition::setConnectionsReadyToReceive()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    apex_assert_hard(areConnections(Connection::State::DONE, Connection::State::NOT_INITIALIZED));

    for(ConnectionWeakPtr c : connections_) {
        c.lock()->setState(Connection::State::READY_TO_RECEIVE);
    }
}
