/// HEADER
#include <csapex/signal/slot.h>

/// COMPONENT
#include <csapex/signal/event.h>
#include <csapex/signal/signal.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Slot::Slot(std::function<void()> callback, const UUID &uuid, bool active)
    : Connectable(uuid), callback_([callback](const TokenConstPtr&){callback();}), active_(active)
{
    setType(connection_types::makeEmpty<connection_types::Signal>());
}
Slot::Slot(std::function<void(const TokenConstPtr&)> callback, const UUID &uuid, bool active)
    : Connectable(uuid), callback_(callback), active_(active)
{
    setType(connection_types::makeEmpty<connection_types::Signal>());
}

Slot::~Slot()
{
}

void Slot::reset()
{
    setSequenceNumber(0);
}

bool Slot::isConnectionPossible(Connectable* other_side)
{
    if(!other_side->canOutput()) {
        std::cerr << "cannot connect, other side can't output" << std::endl;
        return false;
    }

    return other_side->isConnectionPossible(this);
}

bool Slot::acknowledgeConnection(Connectable* other_side)
{
    Event* target = dynamic_cast<Event*>(other_side);

    sources_.push_back(target);

    connected();

    return true;
}

void Slot::removeConnection(Connectable* other_side)
{
    std::vector<Event*>::iterator pos = std::find(sources_.begin(), sources_.end(), other_side);
    if(pos != sources_.end()) {
        sources_.erase(pos);

        connection_removed_to(this);
    }
}

void Slot::enable()
{
    Connectable::enable();
    //    if(isConnected() && !getSource()->isEnabled()) {
    //        getSource()->enable();
    //    }
}

void Slot::disable()
{
    Connectable::disable();

    notifyMessageProcessed();
}

void Slot::removeAllConnectionsNotUndoable()
{
    for(std::vector<Event*>::iterator i = sources_.begin(); i != sources_.end();) {
        Connectable* target = *i;
        target->removeConnection(this);
        i = sources_.erase(i);
    }

    disconnected(this);
}


bool Slot::canConnectTo(Connectable* other_side, bool /*move*/) const
{
    Event* trigger = dynamic_cast<Event*>(other_side);
    return trigger;
}

bool Slot::targetsCanBeMovedTo(Connectable* other_side) const
{
    for(Event* trigger : sources_) {
        if(!trigger->canConnectTo(other_side, true)/* || !canConnectTo(*it)*/) {
            return false;
        }
    }
    return true;
}

bool Slot::isConnected() const
{
    return !sources_.empty();
}

void Slot::connectionMovePreview(Connectable *other_side)
{
    for(Event* trigger : sources_) {
        connectionInProgress(trigger, other_side);
    }
}


void Slot::validateConnections()
{
}

std::vector<Event*> Slot::getSources() const
{
    return sources_;
}

void Slot::trigger(Event* source, TokenConstPtr token)
{
    current_token_ = token,
    triggered(source);
}

void Slot::handleEvent()
{
    apex_assert_hard(current_token_);

    // do the work
    if(isEnabled() || isActive()) {
        callback_(current_token_);
    }

    current_token_.reset();
}

void Slot::notifyMessageProcessed()
{
    Connectable::notifyMessageProcessed();

    for(Event* trigger : sources_) {
        trigger->notifyMessageProcessed();
    }
}

bool Slot::isActive() const
{
    return active_;
}
