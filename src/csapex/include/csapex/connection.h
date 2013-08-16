#ifndef CONNECTION_H
#define CONNECTION_H

/// COMPONENT
#include <csapex/selectable.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QObject>
#include <vector>

namespace csapex
{

/// FORWARD DECLARATION
class Connector;
class ConnectorOut;
class ConnectorIn;

class Connection : public QObject, public Selectable {

    Q_OBJECT

public:
    typedef boost::shared_ptr<Connection> Ptr;

public:
    static const Connection::Ptr NullPtr;
    static const int activity_marker_max_lifetime_;

public:
    Connection(ConnectorOut* from, ConnectorIn* to);

    Connector* from() const;
    Connector* to() const;
    int id() const;

    bool contains(Connector* c) const;

    int activity() const;
    void tick();

private Q_SLOTS:
    void messageSentEvent();

public:
    bool operator == (const Connection& c) const;

protected:
    Connection(Connector* from, Connector* to);

protected:
    Connector* from_;
    Connector* to_;

    int id_;

    int message_count;

    static int next_connection_id_;
};

}

#endif // CONNECTION_H
