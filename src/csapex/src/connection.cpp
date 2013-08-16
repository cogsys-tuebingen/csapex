/// HEADER
#include <csapex/connection.h>

/// COMPONENT
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>

using namespace csapex;

int Connection::next_connection_id_ = 0;
const int Connection::activity_marker_max_lifetime_ = 10;

const Connection::Ptr Connection::NullPtr;

Connection::Connection(ConnectorOut *from, ConnectorIn *to)
    : from_(from), to_(to), id_(next_connection_id_++), message_count(0)
{
    std::cout << "make connection between " << from->UUID() << " and " << to->UUID() << std::endl;
    QObject::connect(from_, SIGNAL(messageSent(ConnectorOut*)), this, SLOT(messageSentEvent()));
}

Connection::Connection(Connector *from, Connector *to)
    : from_(from), to_(to), id_(next_connection_id_++), message_count(0)
{
    std::cout << "make raw connection between " << from->UUID() << " and " << to->UUID() << std::endl;
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
