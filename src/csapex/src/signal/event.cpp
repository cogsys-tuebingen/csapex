/// HEADER
#include <csapex/signal/event.h>

/// COMPONENT
#include <csapex/signal/slot.h>
#include <csapex/utility/timer.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/message_traits.h>
#include <csapex/msg/no_message.h>
#include <csapex/msg/any_message.h>

/// SYSTEM

#include <iostream>

using namespace csapex;

Event::Event(const UUID& uuid)
    : StaticOutput(uuid)
{
    setType(connection_types::makeEmpty<connection_types::AnyMessage>());
}

Event::~Event()
{
}

void Event::reset()
{
    setSequenceNumber(0);
}

void Event::trigger()
{
    TokenConstPtr token(new connection_types::NoMessage);
    triggerWith(token);
}

void Event::triggerWith(TokenConstPtr token)
{
    addMessage(token);
//    commitMessages();
//    publish();
    ++count_;

    triggered();
}

