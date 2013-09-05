#ifndef CONNECTION_H
#define CONNECTION_H

/// COMPONENT
#include <csapex/selectable.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QObject>
#include <vector>
#include <QPoint>

namespace csapex
{

class Connection : public QObject, public Selectable {

    Q_OBJECT

    friend class command::AddFulcrum;
    friend class command::MoveFulcrum;
    friend class command::DeleteFulcrum;
    friend class GraphIO;
    friend class Graph;

public:
    typedef boost::shared_ptr<Connection> Ptr;

public:
    static const Connection::Ptr NullPtr;
    static const int activity_marker_max_lifetime_;

    friend std::ostream& operator << (std::ostream& out, const Connection& c) {
        out << "Connection: [" << c.from() << " / " << c.to() << "]";
        return out;
    }

public:
    Connection(ConnectorOut* from, ConnectorIn* to);
    Connection(ConnectorOut* from, ConnectorIn* to, int id);

    Connector* from() const;
    Connector* to() const;
    int id() const;

    bool contains(Connector* c) const;

    int activity() const;
    void tick();

private Q_SLOTS:
    void messageSentEvent();

Q_SIGNALS:
    void fulcrum_added(Connection*);
    void fulcrum_moved(Connection*);
    void fulcrum_deleted(Connection*);

public:
    bool operator == (const Connection& c) const;

    std::vector<QPoint> getFulcrums() const;
    int getFulcrumCount() const;
    QPoint getFulcrum(int fulcrum_id);

protected:
    Connection(Connector* from, Connector* to);

private:
    /// COMMANDS
    void addFulcrum(int subsection, const QPoint& pos);
    void moveFulcrum(int fulcrum_id, const QPoint& pos);
    void deleteFulcrum(int fulcrum_id);

protected:
    Connector* from_;
    Connector* to_;

    std::vector<QPoint> fulcrums_;

    int id_;

    int message_count;

    static int next_connection_id_;
};

}

#endif // CONNECTION_H
