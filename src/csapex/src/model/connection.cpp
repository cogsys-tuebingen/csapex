/// HEADER
#include <csapex/model/connection.h>

/// COMPONENT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/msg/dynamic_output.h>
#include <csapex/msg/output_transition.h>
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
    : from_(from), to_(to), id_(next_connection_id_++), state_(State::NOT_INITIALIZED)
{
    is_dynamic_ = from_->isDynamic() || to_->isDynamic();

    apex_assert_hard(from->isOutput());
    apex_assert_hard(to->isInput());
    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(Connectable *from, Connectable *to, int id)
    : from_(from), to_(to), id_(id), state_(State::NOT_INITIALIZED)
{
    is_dynamic_ = from_->isDynamic() || to_->isDynamic();

    apex_assert_hard(from->isOutput());
    apex_assert_hard(to->isInput());
    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(Output *from, Input *to)
    : from_(from), to_(to), id_(next_connection_id_++), state_(State::NOT_INITIALIZED)
{
    is_dynamic_ = from_->isDynamic() || to_->isDynamic();

    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(Output *from, Input *to, int id)
    : from_(from), to_(to), id_(id), state_(State::NOT_INITIALIZED)
{
    is_dynamic_ = from_->isDynamic() || to_->isDynamic();

    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}


Connection::Connection(Trigger *from, Slot *to)
    : from_(from), to_(to), id_(next_connection_id_++), state_(State::NOT_INITIALIZED)
{
    is_dynamic_ = from_->isDynamic() || to_->isDynamic();

    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(Trigger *from, Slot *to, int id)
    : from_(from), to_(to), id_(id), state_(State::NOT_INITIALIZED)
{
    is_dynamic_ = from_->isDynamic() || to_->isDynamic();

    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}

ConnectionTypeConstPtr Connection::getMessage() const
{
    apex_assert_hard(message_ != nullptr);
    return message_;
}

void Connection::notifyMessageSet()
{
    new_message();
}

void Connection::notifyMessageProcessed()
{
    std::cerr << "notify connection " <<  from_->getUUID() << " => " << to_->getUUID() << std::endl;
    Output* o = dynamic_cast<Output*>(from_);
    if(o) {
        o->getTransition()->notifyMessageProcessed();
    }
}

void Connection::setMessage(const ConnectionTypeConstPtr &msg)
{
    apex_assert_hard(isEnabled());
    apex_assert_hard(msg != nullptr);
    message_ = msg;
    setState(State::UNREAD);
}

bool Connection::isEnabled() const
{
    return from()->isEnabled() && to()->isEnabled();
}

Connection::State Connection::getState() const
{
    return state_;
}

void Connection::setState(State s)
{
    std::cerr << "SET connection " <<  from_->getUUID() << " => " << to_->getUUID() << ": ";
    switch (s) {
    case State::READY_TO_RECEIVE:
        std::cerr << "ready to receive";
        break;
    case State::UNREAD:
        std::cerr << "unread";
        apex_assert_hard(message_ != nullptr);
        break;
    case State::READ:
        std::cerr << "read";
        apex_assert_hard(message_ != nullptr);
        break;
    default:
        break;
    }
    std::cerr << std::endl;

    state_ = s;
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
