#ifndef CONNECTION_H
#define CONNECTION_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <memory>
#include <QObject>
#include <vector>
#include <QPoint>
#include <deque>
#include <boost/signals2.hpp>
#include <mutex>
#include <condition_variable>

namespace csapex
{

class Connection : public QObject {

    Q_OBJECT

    friend class command::AddFulcrum;
    friend class command::MoveFulcrum;
    friend class command::ModifyFulcrum;
    friend class command::DeleteFulcrum;
    friend class GraphIO;
    friend class Graph;
    friend class Fulcrum;

public:
    typedef std::shared_ptr<Connection> Ptr;

public:
    friend std::ostream& operator << (std::ostream& out, const Connection& c) {
        out << "Connection: [" << c.from() << " / " << c.to() << "]";
        return out;
    }

public:
    Connection(Connectable* from, Connectable* to);
    Connection(Connectable* from, Connectable* to, int id);

    Connection(Output* from, Input* to);
    Connection(Output* from, Input* to, int id);
    Connection(Trigger* from, Slot* to);
    Connection(Trigger* from, Slot* to, int id);

    Connectable* from() const;
    Connectable* to() const;
    int id() const;

    bool contains(Connectable* c) const;

    void tick();

    bool acceptsMessages() const;
    void addMessage(const ConnectionTypeConstPtr& msg);
    void commitMessages();
    void freeMessages();
    int countCommittedMessages() const;
    ConnectionTypeConstPtr takeMessage();

public:
    boost::signals2::signal<void()> messages_committed_;

private Q_SLOTS:
    void messageSentEvent();

Q_SIGNALS:
    void fulcrum_added(Fulcrum*);
    void fulcrum_moved(Fulcrum*,bool dropped);
    void fulcrum_moved_handle(Fulcrum*,bool dropped, int which);
    void fulcrum_type_changed(Fulcrum*,int type);
    void fulcrum_deleted(Fulcrum*);

public:
    bool operator == (const Connection& c) const;

    std::vector<FulcrumPtr> getFulcrums() const;
    int getFulcrumCount() const;
    FulcrumPtr getFulcrum(int fulcrum_id);

private:
    /// COMMANDS
    void addFulcrum(int fulcrum_id, const QPointF& pos, int type, const QPointF& handle_in=QPointF(-10.0, 0.0), const QPointF& handle_out=QPointF(10.0, 0.0));
    void modifyFulcrum(int fulcrum_id, int type, const QPointF& handle_in=QPointF(-10.0, 0.0), const QPointF& handle_out=QPointF(10.0, 0.0));
    void moveFulcrum(int fulcrum_id, const QPointF &pos, bool dropped);
    void deleteFulcrum(int fulcrum_id);

protected:
    Connectable* from_;
    Connectable* to_;
    int id_;
    bool is_dynamic_;

    std::vector<FulcrumPtr> fulcrums_;

    enum class State { COLLECTING, COMMITTED, LATCHING };
    std::list<ConnectionTypeConstPtr> message_queue_;
    std::deque<ConnectionTypeConstPtr> committed_messages_;
    State message_state_;
    mutable std::mutex messages_mutex_;

    int dimensionality_;

    static int next_connection_id_;
};

}

#endif // CONNECTION_H
