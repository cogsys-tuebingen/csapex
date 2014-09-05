/// HEADER
#include <csapex/model/connection.h>

/// COMPONENT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/core/settings.h>
#include <csapex/model/fulcrum.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <cmath>

using namespace csapex;

int Connection::next_connection_id_ = 0;

Connection::Connection(Output *from, Input *to)
    : from_(from), to_(to), id_(next_connection_id_++)
{
    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(Connectable *from, Connectable *to)
    : from_(from), to_(to), id_(next_connection_id_++)
{
    apex_assert_hard(from->isOutput());
    apex_assert_hard(to->isInput());
    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(Output *from, Input *to, int id)
    : from_(from), to_(to), id_(id)
{
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
