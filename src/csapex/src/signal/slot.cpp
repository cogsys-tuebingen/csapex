/// HEADER
#include <csapex/signal/slot.h>

/// COMPONENT
#include <csapex/signal/event.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/any_message.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Slot::Slot(std::function<void()> callback, const UUID &uuid, bool active)
    : Input(uuid), callback_([callback](const TokenConstPtr&){callback();}), active_(active)
{
    setType(connection_types::makeEmpty<connection_types::AnyMessage>());
}
Slot::Slot(std::function<void(const TokenConstPtr&)> callback, const UUID &uuid, bool active)
    : Input(uuid), callback_(callback), active_(active)
{
    setType(connection_types::makeEmpty<connection_types::AnyMessage>());
}

Slot::~Slot()
{
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


void Slot::inputMessage(TokenConstPtr token)
{
    Input::inputMessage(token);

    triggered();
}

void Slot::handleEvent()
{
    apex_assert_hard(message_);

    // do the work
    if(isEnabled() || isActive()) {
        callback_(message_);
    }

    message_.reset();
}

bool Slot::isActive() const
{
    return active_;
}


bool Slot::canConnectTo(Connectable* other_side, bool move) const
{
    return Connectable::canConnectTo(other_side, move);
}
