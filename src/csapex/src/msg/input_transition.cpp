/// HEADER
#include <csapex/msg/input_transition.h>

/// COMPONENT
#include <csapex/model/node_worker.h>
#include <csapex/model/connection.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>

/// SYSTEM
#include <sstream>

using namespace csapex;

InputTransition::InputTransition(NodeWorker* node)
    : Transition(node)
{

}

void InputTransition::connectionAdded(Connection *connection)
{
    connection->new_message.connect([this]() {
        //        std::cerr << "new message in " << node_->getUUID() << std::endl;
        //        update();
        node_->triggerCheckInputs();
    });
}

void InputTransition::update()
{
    if(!node_->canProcess()) {
        return;
    }

    if(node_->isSource()) {
        apex_assert_hard(node_->getState() == NodeWorker::State::IDLE);
        apex_assert_hard(connections_.empty());
        //fire(); -> instead of tick!!!!
    } else {
        if(!isConnection(Connection::State::READY_TO_RECEIVE) &&
                !isConnection(Connection::State::NOT_INITIALIZED) &&
                isConnection(Connection::State::UNREAD)) {
            if(node_->getState() == NodeWorker::State::IDLE) {
                fire();
            }
        }
    }
}

void InputTransition::notifyMessageProcessed()
{
    //todo: dynamic output -> lock connected input transitions' notifyMessageProcessed capability
    if(!isConnection(Connection::State::UNREAD)) {
        std::cerr << "notify input transition " <<  node_->getUUID() << std::endl;
        std::lock_guard<std::recursive_mutex> lock(sync);
        for(auto& connection : connections_) {
            ConnectionPtr c = connection.lock();
            c->notifyMessageProcessed();
        }
    }
    //    update();
}

void InputTransition::fire()
{
    std::lock_guard<std::recursive_mutex> lock(sync);
    apex_assert_hard(node_->canProcess());
    apex_assert_hard(node_->getState() == NodeWorker::State::IDLE);
    apex_assert_hard(!isConnection(Connection::State::NOT_INITIALIZED));
    apex_assert_hard(!isConnection(Connection::State::READY_TO_RECEIVE));
    apex_assert_hard(connections_.empty() || !areConnections(Connection::State::READ));

    std::cerr << "fire " <<  node_->getUUID() << std::endl;

    for(Input* input : node_->getMessageInputs()) {
        std::cerr << "input message from " <<  node_->getUUID() << " -> " << input->getUUID() << std::endl;

        if(input->isConnected()) {
            auto connections = input->getConnections();
            apex_assert_hard(connections.size() == 1);
            auto connection = connections.front().lock();
            apex_assert_hard(connection->getState() == Connection::State::READ ||
                             connection->getState() == Connection::State::UNREAD);
            auto msg = connection->getMessage();
            apex_assert_hard(msg != nullptr);
            input->inputMessage(msg);
        } else {
            input->inputMessage(connection_types::makeEmpty<connection_types::NoMessage>());
        }
    }

    for(auto& connection : connections_) {
        ConnectionPtr c = connection.lock();
        c->setState(Connection::State::READ);
    }

    std::cerr << "-> process " <<  node_->getUUID() << std::endl;

    apex_assert_hard(node_->getState() == NodeWorker::State::IDLE);
    node_->triggerProcess();
}
