/// HEADER
#include <csapex/signal/slot.h>

/// COMPONENT
#include <csapex/signal/event.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/no_message.h>
#include <csapex/model/connection.h>
#include <csapex/model/node_handle.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Slot::Slot(std::function<void()> callback, const UUID &uuid, bool active, bool blocking, ConnectableOwnerWeakPtr owner)
    : Input(uuid, owner), callback_([callback](Slot*, const TokenPtr&){callback();}), active_(active), blocking_(blocking), guard_(-1)
{
    setType(connection_types::makeEmpty<connection_types::AnyMessage>());
}
Slot::Slot(std::function<void(const TokenPtr&)> callback, const UUID &uuid, bool active, bool blocking, ConnectableOwnerWeakPtr owner)
    : Input(uuid, owner), callback_([callback](Slot*, const TokenPtr& token){callback(token);}), active_(active), blocking_(blocking), guard_(-1)
{
    setType(connection_types::makeEmpty<connection_types::AnyMessage>());
}

Slot::Slot(std::function<void(Slot*, const TokenPtr&)> callback, const UUID &uuid, bool active, bool blocking, ConnectableOwnerWeakPtr owner)
    : Input(uuid, owner), callback_(callback), active_(active), blocking_(blocking), guard_(-1)
{
    setType(connection_types::makeEmpty<connection_types::AnyMessage>());
}

Slot::~Slot()
{
    guard_ = 0xDEADBEEF;
}

int Slot::maxConnectionCount() const
{
    return -1;
}

void Slot::reset()
{
    setSequenceNumber(0);
}

void Slot::enable()
{
    Connectable::enable();

    tryNextToken();
}

void Slot::disable()
{
    Connectable::disable();

    notifyMessageProcessed();
}


void Slot::setToken(TokenPtr token)
{
    apex_assert_hard(getType()->canConnectTo(token->getTokenData().get()));

//    Input::setToken(token);

    {
        std::unique_lock<std::mutex> lock(message_mutex_);
        if(!message_) {
            message_ = token;
        }
    }
    count_++;
    message_set(this);

    token_set(token);

    apex_assert_hard(guard_ == -1);
    triggered();
}

void Slot::notifyMessageAvailable(Connection* connection)
{
    message_available(connection);

    {
        std::unique_lock<std::recursive_mutex> lock(available_connections_mutex_);
        available_connections_.push_back(connection);
    }

    tryNextToken();
}

void Slot::notifyMessageProcessed()
{
    message_processed(shared_from_this());

    Connection* front = nullptr;
    {
        std::unique_lock<std::recursive_mutex> lock(available_connections_mutex_);
        if(!available_connections_.empty()) {
            front = available_connections_.front();
            available_connections_.pop_front();
        }
    }
    if(front) {
        front->setTokenProcessed();
    }

    tryNextToken();
}


void Slot::tryNextToken()
{
    if(!isEnabled() && !isActive()) {
        std::vector<Connection*> connections;
        std::unique_lock<std::recursive_mutex> lock(available_connections_mutex_);
        while(!available_connections_.empty()) {
            Connection* c = available_connections_.front();
            connections.push_back(c);
            available_connections_.pop_front();
        }
        lock.unlock();

        for(Connection* c : connections) {
            c->setState(Connection::State::READ);
            c->setTokenProcessed();
        }


    }

    if(isEnabled() || isActive()) {
        std::unique_lock<std::recursive_mutex> lock(available_connections_mutex_);
        if(!message_ && !available_connections_.empty()) {
            auto* current_connection = available_connections_.front();
            TokenPtr token = current_connection->readToken();
            lock.unlock();

            setToken(token);
        }
    }
}

void Slot::handleEvent()
{
    {
        std::unique_lock<std::mutex> lock(message_mutex_);
        apex_assert_hard(message_);

        // do the work
        if(isEnabled() || isActive()) {
            auto msg_copy = message_;
            lock.unlock();

            if(!std::dynamic_pointer_cast<connection_types::NoMessage const>(message_->getTokenData())) {
                apex_assert_hard(guard_ == -1);
                try {
                    callback_(this, msg_copy);
                } catch(const std::exception& e) {
                    std::cerr << "slot " << getUUID() << " has thrown an exception: " << e.what() << std::endl;

                    if(NodeHandlePtr node = std::dynamic_pointer_cast<NodeHandle>(getOwner())) {
                        node->setError(e.what());
                    }
                }
            } else {
                notifyEventHandled();
                return;
            }
        }
    }

    if(blocking_) {
        notifyEventHandled();
    }
}

void Slot::notifyEventHandled()
{
    {
        std::unique_lock<std::mutex> lock(message_mutex_);
        message_.reset();
    }
//    for(auto connection : connections_) {
//        if(connection->getState() == Connection::State::UNREAD) {
//            return;
//        }
//    }
    notifyMessageProcessed();
}

bool Slot::isActive() const
{
    return active_;
}

bool Slot::isBlocking() const
{
    return blocking_;
}

bool Slot::isSynchronous() const
{
    return false;
}

void Slot::addStatusInformation(std::stringstream &status_stream) const
{
    status_stream << ", blocking: " << isBlocking();
}
