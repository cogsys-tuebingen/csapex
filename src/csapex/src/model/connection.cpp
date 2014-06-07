/// HEADER
#include <csapex/model/connection.h>

/// COMPONENT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/core/settings.h>
#include <csapex/model/fulcrum.h>

/// SYSTEM
#include <cmath>

using namespace csapex;

int Connection::next_connection_id_ = 0;

Connection::Connection(ConnectorOut *from, ConnectorIn *to)
    : from_(from), to_(to), id_(next_connection_id_++), message_count(0)
{
    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(Connectable *from, Connectable *to)
    : from_(from), to_(to), id_(next_connection_id_++), message_count(0)
{
    assert(from->isOutput());
    assert(to->isInput());
    QObject::connect(from_, SIGNAL(messageSent(Connectable*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(ConnectorOut *from, ConnectorIn *to, int id)
    : from_(from), to_(to), id_(id), message_count(0)
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

int Connection::activity() const
{
    return std::ceil(message_count);
}

void Connection::messageSentEvent()
{
    message_count = std::min(static_cast<double> (Settings::activity_marker_max_lifetime_), message_count + 1);
}

void Connection::tick()
{
    message_count = std::max(0.0, message_count - 0.1);
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

void Connection::addFulcrum(int subsection, const QPointF &pos, int type)
{
    // create the new fulcrum
    Fulcrum::Ptr f(new Fulcrum(this, pos, type));
    f->setId(subsection);

    // update the ids of the later fulcrums
    std::vector<Fulcrum::Ptr>::iterator index = fulcrums_.begin() + subsection;
    for(std::vector<Fulcrum::Ptr>::iterator it = index; it != fulcrums_.end(); ++it) {
        (*it)->setId((*it)->id() + 1);
    }

    fulcrums_.insert(index, f);

    QObject::connect(f.get(), SIGNAL(moved(Fulcrum*,bool)), this, SIGNAL(fulcrum_moved(Fulcrum*,bool)));

    Q_EMIT fulcrum_added(f.get());
}

void Connection::moveFulcrum(int fulcrum_id, const QPointF &pos, bool dropped)
{
    fulcrums_[fulcrum_id]->move(pos, dropped);
}

void Connection::deleteFulcrum(int fulcrum_id)
{
    Q_EMIT fulcrum_deleted((fulcrums_[fulcrum_id]).get());

    // update the ids of the later fulcrums
    std::vector<Fulcrum::Ptr>::iterator index = fulcrums_.begin() + fulcrum_id;
    for(std::vector<Fulcrum::Ptr>::iterator it = index; it != fulcrums_.end(); ++it) {
        (*it)->setId((*it)->id() - 1);
    }

    fulcrums_.erase(index);
}
