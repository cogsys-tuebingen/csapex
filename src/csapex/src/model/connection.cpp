/// HEADER
#include <csapex/model/connection.h>

/// COMPONENT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/msg/dynamic_output.h>
#include <csapex/msg/output_transition.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/event.h>
#include <csapex/core/settings.h>
#include <csapex/model/fulcrum.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/no_message.h>

/// SYSTEM
#include <cmath>

using namespace csapex;

int Connection::next_connection_id_ = 0;

Connection::Connection(Output *from, Input *to)
    : Connection(from, to, next_connection_id_++)
{
}

Connection::Connection(Output *from, Input *to, int id)
    : from_(from), to_(to), id_(id),
      active_(false), detached_(false),
      state_(State::NOT_INITIALIZED)
{
    is_dynamic_ = from_->isDynamic() || to_->isDynamic();

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
    if(c == from_) {
        from_ = nullptr;
        detached_ = true;

    } else if(c == to_) {
        to_ = nullptr;
        detached_ = true;
    }
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
    return message_ && message_->isActive();
}

void Connection::notifyMessageSet()
{
    if(detached_) {
        return;
    }
    to_->notifyMessageAvailable(this);
}

void Connection::setTokenProcessed()
{
    {
        std::unique_lock<std::recursive_mutex> lock(sync);
        setState(State::DONE);
    }
    if(detached_) {
        return;
    }
    from_->setMessageProcessed();
}

void Connection::setToken(const TokenPtr &token)
{
    {
        TokenPtr msg = token->clone();

        std::unique_lock<std::recursive_mutex> lock(sync);
        apex_assert_hard(msg != nullptr);
        apex_assert_hard(state_ == State::NOT_INITIALIZED);

        bool msg_active = msg->isActive();
        if(!isActive() && msg_active) {
            // remove active flag if the connection is inactive
            msg->setActive(false);
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

Output* Connection::from() const
{
    return from_;
}

Input *Connection::to() const
{
    return to_;
}

int Connection::id() const
{
    return id_;
}

bool Connection::contains(Connectable *c) const
{
    return from_ == c || to_ == c;
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
    out << "Connection: [" << c.from() << " / " << c.to() << "]";
    return out;
}
