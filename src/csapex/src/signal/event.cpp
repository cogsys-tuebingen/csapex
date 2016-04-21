/// HEADER
#include <csapex/signal/event.h>

/// COMPONENT
#include <csapex/signal/slot.h>
#include <csapex/signal/signal.h>
#include <csapex/utility/timer.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/message_traits.h>

/// SYSTEM

#include <iostream>

using namespace csapex;

Event::Event(const UUID& uuid)
    : Connectable(uuid)
{
    setType(connection_types::makeEmpty<connection_types::Signal>());
}

Event::~Event()
{
}

void Event::reset()
{
    setSequenceNumber(0);
}

int Event::noTargets()
{
    return targets_.size();
}

std::vector<Slot*> Event::getTargets() const
{
    return targets_;
}

void Event::removeConnection(Connectable* other_side)
{
    for(std::vector<Slot*>::iterator i = targets_.begin(); i != targets_.end();) {
        if(*i == other_side) {
            other_side->removeConnection(this);

            i = targets_.erase(i);

            connection_removed_to(this);
            return;

        } else {
            ++i;
        }
    }
}

void Event::removeAllConnectionsNotUndoable()
{
    for(std::vector<Slot*>::iterator i = targets_.begin(); i != targets_.end();) {
        (*i)->removeConnection(this);
        i = targets_.erase(i);
    }

    disconnected(this);
}

void Event::trigger()
{
    {
        std::unique_lock<std::recursive_mutex> lock(targets_running_mtx_);
        apex_assert_hard(targets_running_.empty());
        for(Slot* s : targets_) {
            targets_running_[s] = true;
        }
    }

    triggered();

    for(Slot* s : targets_) {
        try {
            s->trigger(this);
        } catch(const std::exception& e) {
            std::cerr << "triggering slot " << s->getLabel()  << " failed: " << e.what();
        }
    }
    ++count_;
}

void Event::signalHandled(Slot *slot)
{
    std::unique_lock<std::recursive_mutex> lock(targets_running_mtx_);
    targets_running_.erase(slot);

    if(targets_running_.empty()) {
        all_signals_handled();
    }
}

bool Event::isBeingProcessed() const
{
    std::unique_lock<std::recursive_mutex> lock(targets_running_mtx_);
    return !targets_running_.empty();
}

void Event::disable()
{
    Connectable::disable();
}

bool Event::isConnectionPossible(Connectable *other_side)
{
    Slot* slot = dynamic_cast<Slot*>(other_side);
    if(!slot) {
        return false;
    }
    return true;
}

bool Event::connect(Slot *slot)
{
    apex_assert_hard(slot);

    if(!slot->acknowledgeConnection(this)) {
        std::cerr << "cannot connect, slot doesn't acknowledge" << std::endl;
        return false;
    }

    targets_.push_back(slot);

    validateConnections();
    return true;
}

bool Event::canConnectTo(Connectable* other_side, bool move) const
{
    if(!Connectable::canConnectTo(other_side, move)) {
        return false;
    } else {
        for (Slot* target : targets_) {
            if(target == other_side) {
                return false;
            }
        }
    }
    return true;
}


bool Event::targetsCanBeMovedTo(Connectable* other_side) const
{
    for(Slot* slot : targets_) {
        if(!slot->canConnectTo(other_side, true)/* || !canConnectTo(*it)*/) {
            return false;
        }
    }
    return true;
}

bool Event::isConnected() const
{
    return targets_.size() > 0;
}

void Event::connectionMovePreview(Connectable *other_side)
{
    for(Slot* slot : targets_) {
        connectionInProgress(slot, other_side);
    }
}

void Event::validateConnections()
{
    for(Slot* target : targets_) {
        target->validateConnections();
    }
}
