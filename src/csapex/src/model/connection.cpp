/// HEADER
#include <csapex/model/connection.h>

/// COMPONENT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>

using namespace csapex;

int Connection::next_connection_id_ = 0;
const int Connection::activity_marker_max_lifetime_ = 10;

const Connection::Ptr Connection::NullPtr;

Connection::Connection(ConnectorOut *from, ConnectorIn *to)
    : from_(from), to_(to), id_(next_connection_id_++), message_count(0)
{
    QObject::connect(from_, SIGNAL(messageSent(Connector*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(Connector *from, Connector *to)
    : from_(from), to_(to), id_(next_connection_id_++), message_count(0)
{
    QObject::connect(from_, SIGNAL(messageSent(Connector*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(ConnectorOut *from, ConnectorIn *to, int id)
    : from_(from), to_(to), id_(id), message_count(0)
{
    QObject::connect(from_, SIGNAL(messageSent(Connector*)), this, SLOT(messageSentEvent()));
}

Connector* Connection::from() const
{
    return from_;
}

Connector* Connection::to() const
{
    return to_;
}

int Connection::id() const
{
    return id_;
}

bool Connection::contains(Connector *c) const
{
    return from_ == c || to_ == c;
}

bool Connection::operator == (const Connection& c) const
{
    return from_ == c.from() && to_ == c.to();
}

int Connection::activity() const
{
    return message_count / 3;
}

void Connection::messageSentEvent()
{
    message_count = std::min(Connection::activity_marker_max_lifetime_ * 3, message_count + 3);
}

void Connection::tick()
{
    message_count = std::max(0, message_count - 1);
}

std::vector<QPoint> Connection::getFulcrums() const
{
    return fulcrums_;
}

int Connection::getFulcrumCount() const
{
    return fulcrums_.size();
}

QPoint Connection::getFulcrum(int fulcrum_id)
{
    return fulcrums_[fulcrum_id];
}

void Connection::addFulcrum(int subsection, const QPoint &pos)
{
    std::size_t before = fulcrums_.size();
    fulcrums_.insert(fulcrums_.begin() + subsection, pos);
    assert(before == fulcrums_.size() - 1);
    Q_EMIT fulcrum_added(this);
}

void Connection::moveFulcrum(int fulcrum_id, const QPoint &pos)
{
    fulcrums_[fulcrum_id] = pos;
    Q_EMIT fulcrum_moved(this);
}

void Connection::deleteFulcrum(int fulcrum_id)
{
    std::size_t before = fulcrums_.size();
    fulcrums_.erase(fulcrums_.begin() + fulcrum_id);
    assert(before == fulcrums_.size() + 1);
    Q_EMIT fulcrum_deleted(this);
}
