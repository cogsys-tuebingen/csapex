/// HEADER
#include <csapex/signal/slot.h>

/// COMPONENT
#include <csapex/signal/trigger.h>
#include <csapex/signal/signal.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Slot::Slot(std::function<void()> callback, const UUID &uuid, bool active)
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
    Trigger* target = dynamic_cast<Trigger*>(other_side);

    sources_.push_back(target);

    connected();

    return true;
}

void Slot::removeConnection(Connectable* other_side)
{
    std::vector<Trigger*>::iterator pos = std::find(sources_.begin(), sources_.end(), other_side);
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
    for(std::vector<Trigger*>::iterator i = sources_.begin(); i != sources_.end();) {
        Connectable* target = *i;
        target->removeConnection(this);
        i = sources_.erase(i);
    }

    disconnected(this);
}


bool Slot::canConnectTo(Connectable* other_side, bool /*move*/) const
{
    Trigger* trigger = dynamic_cast<Trigger*>(other_side);
    return trigger;
}

bool Slot::targetsCanBeMovedTo(Connectable* other_side) const
{
    for(Trigger* trigger : sources_) {
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
    for(Trigger* trigger : sources_) {
        connectionInProgress(trigger, other_side);
    }
}


void Slot::validateConnections()
{
}

std::vector<Trigger*> Slot::getSources() const
{
    return sources_;
}

void Slot::trigger(Trigger* source)
{
    std::cerr << "slot " << getLabel() << " triggered" << std::endl;
    triggered(source);
}

void Slot::handleTrigger()
{
    // do the work
    if(isEnabled() || isActive()) {
        callback_();
    }
}

void Slot::notifyMessageProcessed()
{
    Connectable::notifyMessageProcessed();

    for(Trigger* trigger : sources_) {
        trigger->notifyMessageProcessed();
    }
}

bool Slot::isActive() const
{
    return active_;
}
