/// HEADER
#include <csapex/model/connection.h>

/// COMPONENT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/core/settings.h>

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

std::vector<Connection::Fulcrum> Connection::getFulcrums() const
{
    return fulcrums_;
}

int Connection::getFulcrumCount() const
{
    return fulcrums_.size();
}

Connection::Fulcrum Connection::getFulcrum(int fulcrum_id)
{
    return fulcrums_[fulcrum_id];
}

void Connection::addFulcrum(int subsection, const QPoint &pos, int type)
{
    std::size_t before = fulcrums_.size();
    fulcrums_.insert(fulcrums_.begin() + subsection, Connection::Fulcrum(pos, type));
    assert(before == fulcrums_.size() - 1);
    Q_EMIT fulcrum_added(this);
}

void Connection::moveFulcrum(int fulcrum_id, const QPoint &pos)
{
    fulcrums_[fulcrum_id].pos = pos;
    Q_EMIT fulcrum_moved(this);
}

void Connection::deleteFulcrum(int fulcrum_id)
{
    std::size_t before = fulcrums_.size();
    fulcrums_.erase(fulcrums_.begin() + fulcrum_id);
    assert(before == fulcrums_.size() + 1);
    Q_EMIT fulcrum_deleted(this);
}
