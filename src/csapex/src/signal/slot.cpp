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

Slot::Slot(std::function<void()> callback, const UUID &uuid, bool active)
    : Input(uuid), callback_([callback](const TokenConstPtr&){callback();}), active_(active), guard_(-1)
{
    setType(connection_types::makeEmpty<connection_types::AnyMessage>());
}
Slot::Slot(std::function<void(const TokenPtr&)> callback, const UUID &uuid, bool active)
    : Input(uuid), callback_(callback), active_(active), guard_(-1)
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
    messageProcessed(this);

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
            callback_(message_);
        }
    }

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
