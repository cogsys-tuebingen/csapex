/// HEADER
#include <csapex/signal/slot.h>

/// COMPONENT
#include <csapex/signal/event.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/no_message.h>
#include <csapex/model/connection.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Slot::Slot(std::function<void()> callback, const UUID &uuid, bool active, bool asynchronous, ConnectableOwnerWeakPtr owner)
    : Input(uuid, owner), callback_([callback](Slot*, const TokenPtr&){callback();}), active_(active), asynchronous_(asynchronous), guard_(-1)
{
    setType(connection_types::makeEmpty<connection_types::AnyMessage>());
}
Slot::Slot(std::function<void(const TokenPtr&)> callback, const UUID &uuid, bool active, bool asynchronous, ConnectableOwnerWeakPtr owner)
    : Input(uuid, owner), callback_([callback](Slot*, const TokenPtr& token){callback(token);}), active_(active), asynchronous_(asynchronous), guard_(-1)
{
    setType(connection_types::makeEmpty<connection_types::AnyMessage>());
}

Slot::Slot(std::function<void(Slot*, const TokenPtr&)> callback, const UUID &uuid, bool active, bool asynchronous, ConnectableOwnerWeakPtr owner)
    : Input(uuid, owner), callback_(callback), active_(active), asynchronous_(asynchronous), guard_(-1)
{
    setType(connection_types::makeEmpty<connection_types::AnyMessage>());
}

Slot::~Slot()
{
    guard_ = 0xDEADBEEF;
}

void Slot::reset()
{
    setSequenceNumber(0);
}


void Slot::disable()
{
    Connectable::disable();

    notifyMessageProcessed();
}


void Slot::setToken(TokenPtr token)
{
    apex_assert_hard(getType()->canConnectTo(token->getTokenData().get()));
    Input::setToken(token);

    token_set(token);

    apex_assert_hard(guard_ == -1);
    triggered();
}

void Slot::notifyMessageAvailable(Connection* connection)
{
    message_available(connection);

    std::unique_lock<std::recursive_mutex> lock(available_connections_mutex_);

    available_connections_.push_back(connection);

    if(!message_) {
        TokenPtr token = available_connections_.front()->readToken();
        lock.unlock();

        setToken(token);
    }
}

void Slot::notifyMessageProcessed()
{
    message_processed(this);

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

    if(isEnabled() || isActive()) {
        std::unique_lock<std::recursive_mutex> lock(available_connections_mutex_);
        if(!available_connections_.empty()) {
            TokenPtr token = available_connections_.front()->readToken();
            lock.unlock();

            setToken(token);
        }
    }
}

void Slot::handleEvent()
{
    apex_assert_hard(message_);

    // do the work
    if(isEnabled() || isActive()) {
        if(!std::dynamic_pointer_cast<connection_types::NoMessage const>(message_->getTokenData())) {
            apex_assert_hard(guard_ == -1);
            try {
                callback_(this, message_);
            } catch(const std::exception& e) {
                std::cerr << "slot " << getUUID() << " has thrown an exception: " << e.what() << std::endl;
            }
        } else {
            notifyEventHandled();
            return;
        }
    }

    if(!asynchronous_) {
        notifyEventHandled();
    }
}

void Slot::notifyEventHandled()
{
    message_.reset();
    notifyMessageProcessed();
}

bool Slot::isActive() const
{
    return active_;
}


bool Slot::canConnectTo(Connectable* other_side, bool move) const
{
    return Connectable::canConnectTo(other_side, move);
}
