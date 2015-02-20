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
        if(output->getState() != Output::State::RECEIVING) {
            return false;
        }
    }
    return true;
}

int OutputTransition::sendMessages()
{
    int connected = 0;
    std::cerr << "commit messages output transition: " << node_->getUUID() << std::endl;
    for(Output* out : node_->getMessageOutputs()) {
        if(out->isConnected()) {
            out->commitMessages();
            ++connected;
        }
    }

    apex_assert_hard(areConnections(Connection::State::READY_TO_RECEIVE));

    std::cerr << "fill first time: " << node_->getUUID() << std::endl;
    fillConnections();

    return connected;
}

void OutputTransition::notifyMessageProcessed()
{
    if(!areConnections(Connection::State::READ)) {
        std::cerr << "there are unread connections at: " << node_->getUUID() << std::endl;
        return;
    }
    std::cerr << "notified output transition: " << node_->getUUID() << std::endl;

    for(Output* out : node_->getMessageOutputs()) {
        out->nextMessage();
    }
    if(areOutputsDone()) {
        std::cerr << "all outputs are done: " << node_->getUUID() << std::endl;
        clear();
    } else {
        std::cerr << "fill again: " << node_->getUUID() << std::endl;
        fillConnections();
    }
}

bool OutputTransition::areOutputsDone() const
{
    for(Output* out : node_->getMessageOutputs()) {
        if(out->getState() != Output::State::DONE) {
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
        Output* out = dynamic_cast<Output*>(connection->from());
        apex_assert_hard(out);
        auto msg = out->getMessage();
        apex_assert_hard(msg);
        connection->setMessage(msg);
    }
}

void OutputTransition::clear()
{
    std::cerr << "clear output transition: " << node_->getUUID() << std::endl;
    for(ConnectionWeakPtr c : connections_) {
        c.lock()->setState(Connection::State::READY_TO_RECEIVE);
    }
    for(Output* output : node_->getMessageOutputs()) {
        output->clear();
    }
    node_->notifyMessagesProcessed();
}

