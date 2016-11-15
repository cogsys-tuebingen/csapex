/// HEADER
#include <csapex/model/connection.h>

/// COMPONENT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/msg/output_transition.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/event.h>
#include <csapex/core/settings.h>
#include <csapex/model/fulcrum.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/no_message.h>
#include <csapex/utility/debug.h>

/// SYSTEM
#include <cmath>
#include <iostream>

using namespace csapex;

int Connection::next_connection_id_ = 0;

Connection::Connection(OutputPtr from, InputPtr to)
    : Connection(from, to, next_connection_id_++)
{
}

Connection::Connection(OutputPtr from, InputPtr to, int id)
    : from_(from), to_(to), id_(id),
      active_(false),
      detached_(false),
      state_(State::NOT_INITIALIZED)
{
    from->enabled_changed.connect(source_enable_changed);
    to->enabled_changed.connect(sink_enabled_changed);

    apex_assert_hard(from->isOutput());
    apex_assert_hard(to->isInput());
}

Connection::~Connection()
{
}

void Connection::detach(Connectable *c)
{
    if(c == from_.get()) {
        from_.reset();
        detached_ = true;

    } else if(c == to_.get()) {
        to_.reset();
        detached_ = true;
    }
}

bool Connection::isDetached() const
{
    return detached_;
}

void Connection::reset()
{
    state_ = Connection::State::NOT_INITIALIZED;
}

TokenPtr Connection::getToken() const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    return message_;
}

TokenPtr Connection::readToken()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    setState(State::READ);
    return message_;
}

bool Connection::holdsActiveToken() const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    return message_ && message_->hasActivityModifier();
}

void Connection::setTokenProcessed()
{
    {
        std::unique_lock<std::recursive_mutex> lock(sync);
        setState(State::DONE);
    }
    APEX_DEBUG_CERR <<*this << " is done" << std::endl;
    notifyMessageProcessed();
}

void Connection::setToken(const TokenPtr &token)
{
    {
        TokenPtr msg = token->clone();

        std::unique_lock<std::recursive_mutex> lock(sync);
        apex_assert_hard(msg != nullptr);
        apex_assert_hard(state_ == State::NOT_INITIALIZED);

        if(!isActive() && msg->hasActivityModifier()) {
            // remove active flag if the connection is inactive
            msg->setActivityModifier(ActivityModifier::NONE);
        }

        message_ = msg;
        if(isSinkEnabled()) {
            setState(State::UNREAD);

        } else {
            setState(State::UNREAD);
            setState(State::READ);
        }
    }

    notifyMessageSet();
}

void Connection::notifyMessageSet()
{
    if(detached_) {
        return;
    }
    to_->notifyMessageAvailable(this);
}


void Connection::notifyMessageProcessed()
{
    if(detached_) {
        return;
    }
    from_->notifyMessageProcessed(this);
}


bool Connection::isActive() const
{
    return active_;
}

void Connection::setActive(bool active)
{
    if(active != active_) {
        active_ = active;
    }
}

bool Connection::isEnabled() const
{
    return isSourceEnabled() && isSinkEnabled();
}

bool Connection::isSourceEnabled() const
{
    return from()->isEnabled();
}

bool Connection::isSinkEnabled() const
{
    return to()->isEnabled();
}


Connection::State Connection::getState() const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    return state_;
}

void Connection::setState(State s)
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    //    switch (s) {
    //    case State::UNREAD:
    //        apex_assert_hard(state_ == State::NOT_INITIALIZED);
    //        apex_assert_hard(message_ != nullptr);
    //        break;
    //    case State::READ:
    //        apex_assert_hard(state_ == State::UNREAD || state_ == State::READ);
    //        apex_assert_hard(message_ != nullptr);
    //        break;
    //    case State::DONE:
    //        apex_assert_hard(/*state_ == State::UNREAD || */state_ == State::READ);
    //        apex_assert_hard(message_ != nullptr);
    //        break;
    //    default:
    //        break;
    //    }

    state_ = s;
}

OutputPtr Connection::from() const
{
    return from_;
}

InputPtr Connection::to() const
{
    return to_;
}

int Connection::id() const
{
    return id_;
}

bool Connection::contains(Connectable *c) const
{
    return from_.get() == c || to_.get() == c;
}

bool Connection::operator == (const Connection& c) const
{
    return from_ == c.from() && to_ == c.to();
}

std::vector<Fulcrum::Ptr> Connection::getFulcrums() const
{
    return fulcrums_;
}

int Connection::getFulcrumCount() const
{
    return fulcrums_.size();
}

Fulcrum::Ptr Connection::getFulcrum(int fulcrum_id)
{
    return fulcrums_[fulcrum_id];
}

void Connection::addFulcrum(int fulcrum_id, const Point &pos, int type, const Point &handle_in, const Point &handle_out)
{
    // create the new fulcrum
    Fulcrum::Ptr fulcrum(new Fulcrum(this, pos, type, handle_in, handle_out));
    Fulcrum* f = fulcrum.get();

    f->setId(fulcrum_id);

    // update the ids of the later fulcrums
    if(fulcrum_id < (int) fulcrums_.size()) {
        std::vector<Fulcrum::Ptr>::iterator index = fulcrums_.begin() + fulcrum_id;
        for(std::vector<Fulcrum::Ptr>::iterator it = index; it != fulcrums_.end(); ++it) {
            (*it)->setId((*it)->id() + 1);
        }
        fulcrums_.insert(index, fulcrum);

    } else {
        fulcrums_.push_back(fulcrum);
    }


    f->moved.connect(fulcrum_moved);
    f->movedHandle.connect(fulcrum_moved_handle);
    f->typeChanged.connect(fulcrum_type_changed);

    fulcrum_added(f);
}

void Connection::modifyFulcrum(int fulcrum_id, int type, const Point &handle_in, const Point &handle_out)
{
    Fulcrum::Ptr f = fulcrums_[fulcrum_id];
    f->setType(type);
    f->moveHandles(handle_in, handle_out, false);
}

void Connection::moveFulcrum(int fulcrum_id, const Point &pos, bool dropped)
{
    fulcrums_[fulcrum_id]->move(pos, dropped);
}

void Connection::deleteFulcrum(int fulcrum_id)
{
    apex_assert_hard(fulcrum_id >= 0 && fulcrum_id < (int) fulcrums_.size());
    fulcrum_deleted((fulcrums_[fulcrum_id]).get());

    // update the ids of the later fulcrums
    std::vector<Fulcrum::Ptr>::iterator index = fulcrums_.begin() + fulcrum_id;
    for(std::vector<Fulcrum::Ptr>::iterator it = index; it != fulcrums_.end(); ++it) {
        (*it)->setId((*it)->id() - 1);
    }

    fulcrums_.erase(index);
}

std::ostream& csapex::operator << (std::ostream& out, const Connection& c) {
    out << "Connection: [" << c.from()->getUUID() << " / " << c.to()->getUUID() << "]";
    return out;
}
