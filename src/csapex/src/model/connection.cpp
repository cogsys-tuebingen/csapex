/// HEADER
#include <csapex/model/connection.h>

/// COMPONENT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/msg/dynamic_output.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/trigger.h>
#include <csapex/core/settings.h>
#include <csapex/model/fulcrum.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <cmath>

using namespace csapex;

int Connection::next_connection_id_ = 0;


Connection::Connection(Connectable *from, Connectable *to)
    : from_(from), to_(to), id_(next_connection_id_++), dimensionality_(-1)
{
    is_dynamic_ = from_->isDynamic() || to_->isDynamic();
    message_state_ = State::COLLECTING;

    apex_assert_hard(from->isOutput());
    apex_assert_hard(to->isInput());
    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(Connectable *from, Connectable *to, int id)
    : from_(from), to_(to), id_(id), dimensionality_(-1)
{
    is_dynamic_ = from_->isDynamic() || to_->isDynamic();
    message_state_ = State::COLLECTING;

    apex_assert_hard(from->isOutput());
    apex_assert_hard(to->isInput());
    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(Output *from, Input *to)
    : from_(from), to_(to), id_(next_connection_id_++), dimensionality_(-1)
{
    is_dynamic_ = from_->isDynamic() || to_->isDynamic();
    message_state_ = State::COLLECTING;
    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(Output *from, Input *to, int id)
    : from_(from), to_(to), id_(id), dimensionality_(-1)
{
    is_dynamic_ = from_->isDynamic() || to_->isDynamic();
    message_state_ = State::COLLECTING;
    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}


Connection::Connection(Trigger *from, Slot *to)
    : from_(from), to_(to), id_(next_connection_id_++), dimensionality_(-1)
{
    is_dynamic_ = from_->isDynamic() || to_->isDynamic();
    message_state_ = State::COLLECTING;
    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(Trigger *from, Slot *to, int id)
    : from_(from), to_(to), id_(id), dimensionality_(-1)
{
    is_dynamic_ = from_->isDynamic() || to_->isDynamic();
    message_state_ = State::COLLECTING;
    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}

Connectable* Connection::from() const
{
    return from_;
}

Connectable* Connection::to() const
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

void Connection::messageSentEvent()
{
}

void Connection::tick()
{
}

bool Connection::acceptsMessages() const
{
    std::unique_lock<std::mutex> lock(messages_mutex_);

    if(message_state_ == State::COMMITTED) {
        return false;
    }

    if(is_dynamic_) {
        return message_queue_.empty();;///true;
    } else {
        return message_queue_.empty();
    }
}

void Connection::addMessage(const ConnectionTypeConstPtr &msg)
{
    std::unique_lock<std::mutex> lock(messages_mutex_);
    message_queue_.push_back(msg);
}

void Connection::commitMessages()
{
    {
    std::unique_lock<std::mutex> lock(messages_mutex_);
//    while(has_committed_msgs_) {
//        can_commit_.wait(lock);
//    }
    message_state_ = State::COMMITTED;
    dimensionality_ = message_queue_.size();

    committed_messages_.clear();
    committed_messages_.assign(message_queue_.begin(), message_queue_.end());

    message_queue_.clear();
    }

    messages_committed_();
}

int Connection::countCommittedMessages() const
{
    std::unique_lock<std::mutex> lock(messages_mutex_);
//    return dimensionality_;
    return committed_messages_.size();
}

ConnectionTypeConstPtr Connection::takeMessage()
{
    assert(dimensionality_ != -1);
    if(committed_messages_.empty()) {
        return connection_types::makeEmpty<connection_types::NoMessage>();
    }

    if(is_dynamic_) {
        auto r = committed_messages_.front();
        committed_messages_.pop_front();
        return r;

    } else {
        return committed_messages_.front();
    }
}

void Connection::freeMessages()
{
    {
        std::unique_lock<std::mutex> lock(messages_mutex_);
//        committed_messages_.clear();
//        dimensionality_ = -1;
        message_state_ = State::LATCHING;
    }


    from_->notifyMessageProcessed();
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

void Connection::addFulcrum(int fulcrum_id, const QPointF &pos, int type, const QPointF &handle_in, const QPointF &handle_out)
{
    // create the new fulcrum
    Fulcrum::Ptr fulcrum(new Fulcrum(this, pos, type, handle_in, handle_out));
    Fulcrum* f = fulcrum.get();

    f->setId(fulcrum_id);

    // update the ids of the later fulcrums
    std::vector<Fulcrum::Ptr>::iterator index = fulcrums_.begin() + fulcrum_id;
    for(std::vector<Fulcrum::Ptr>::iterator it = index; it != fulcrums_.end(); ++it) {
        (*it)->setId((*it)->id() + 1);
    }

    fulcrums_.insert(index, fulcrum);

    QObject::connect(f, SIGNAL(moved(Fulcrum*,bool)), this, SIGNAL(fulcrum_moved(Fulcrum*,bool)));
    QObject::connect(f, SIGNAL(movedHandle(Fulcrum*,bool,int)), this, SIGNAL(fulcrum_moved_handle(Fulcrum*,bool,int)));
    QObject::connect(f, SIGNAL(typeChanged(Fulcrum*,int)), this, SIGNAL(fulcrum_type_changed(Fulcrum*,int)));

    Q_EMIT fulcrum_added(f);
}

void Connection::modifyFulcrum(int fulcrum_id, int type, const QPointF &handle_in, const QPointF &handle_out)
{
    Fulcrum::Ptr f = fulcrums_[fulcrum_id];
    f->setType(type);
    f->moveHandles(handle_in, handle_out, false);
}

void Connection::moveFulcrum(int fulcrum_id, const QPointF &pos, bool dropped)
{
    fulcrums_[fulcrum_id]->move(pos, dropped);
}

void Connection::deleteFulcrum(int fulcrum_id)
{
    apex_assert_hard(fulcrum_id >= 0 && fulcrum_id < (int) fulcrums_.size());
    Q_EMIT fulcrum_deleted((fulcrums_[fulcrum_id]).get());

    // update the ids of the later fulcrums
    std::vector<Fulcrum::Ptr>::iterator index = fulcrums_.begin() + fulcrum_id;
    for(std::vector<Fulcrum::Ptr>::iterator it = index; it != fulcrums_.end(); ++it) {
        (*it)->setId((*it)->id() - 1);
    }

    fulcrums_.erase(index);
}
/// MOC
#include "../../include/csapex/model/moc_connection.cpp"
