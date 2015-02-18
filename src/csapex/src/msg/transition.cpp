/// HEADER
#include <csapex/msg/transition.h>

/// COMPONENT
#include <csapex/model/node_worker.h>
#include <csapex/model/connection.h>
#include <csapex/msg/input.h>

/// SYSTEM
#include <sstream>

using namespace csapex;

Transition::Transition(NodeWorker* node)
    : node_(node), firings_left_(0)
{

}

int Transition::determineNeededFirings() const
{
    std::lock_guard<std::recursive_mutex> lock(sync);

    int necessary_firings = 1;
    for(const auto& connection_ptr : connections_) {
        auto connection = connection_ptr.lock();
        int c = connection->countCommittedMessages();
        if(c <= 0) {
            return 0;
        }

        if(c != necessary_firings) {
            std::cerr << "!!!!!! " << c << std::endl;
            if(necessary_firings == 1) {
                necessary_firings = c;
            } else {
                throw std::runtime_error("cannot mix dynamic outputs of different dimensionalities!");
            }
        }
    }

    return necessary_firings;
}

void Transition::addConnection(ConnectionWeakPtr connection)
{
    std::lock_guard<std::recursive_mutex> lock(sync);
    connections_.push_back(connection);

    ConnectionPtr c = connection.lock();
    c->messages_committed_.connect([this]() {
        std::cerr << "commmit to " << node_->getUUID() << std::endl;
        fireIfPossible();
    });
}

void Transition::notifyMessageProcessed()
{
    std::lock_guard<std::recursive_mutex> lock(sync);

    if(firings_left_ > 0) {
        std::cerr << "fire again " << node_->getUUID() << std::endl;
        fire();
    }

    if(firings_left_ == 0) {
        firings_left_ = -1;
        int i = 0;
        for(const auto& c : connections_) {
            ConnectionPtr connection = c.lock();
            if(connection) {
                std::cerr << i++ << " / " << connections_.size() << ": free " <<  node_->getUUID() << " -> " << connection->from()->getUUID() << std::endl;
                connection->freeMessages();
            }
        }
    }
}

void Transition::fireIfPossible()
{
    std::lock_guard<std::recursive_mutex> lock(sync);

    firings_left_ = determineNeededFirings();

    if(firings_left_ > 0) {
        fire();
    }
}

void Transition::fire()
{
    std::lock_guard<std::recursive_mutex> lock(sync);

    std::cerr << "fire " <<  node_->getUUID() << std::endl;

    for(const auto& c : connections_) {
        ConnectionPtr connection = c.lock();
        if(connection) {
            Input* input = dynamic_cast<Input*>(connection->to());
            if(input) {
                std::cerr << "fire " <<  node_->getUUID() << " -> " << input->getUUID() << std::endl;
                auto msg = connection->takeMessage();
                apex_assert_hard(msg != nullptr);
                input->inputMessage(msg);
            }
        }
    }
    --firings_left_;

    node_->triggerProcess();
}

void Transition::removeConnection(ConnectionWeakPtr connection)
{
    std::lock_guard<std::recursive_mutex> lock(sync);
    for(auto it = connections_.begin(); it != connections_.end(); ) {
        const auto& c = *it;
        if(c.lock() == connection.lock()) {
            it = connections_.erase(it);
        } else {
            ++it;
        }
    }
}

NodeWorker* Transition::getNode() const
{
    return node_;
}
