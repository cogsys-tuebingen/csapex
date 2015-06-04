#ifndef CONNECTION_H
#define CONNECTION_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/data/point.h>

/// SYSTEM
#include <memory>
#include <vector>
#include <deque>
#include <boost/signals2.hpp>
#include <mutex>
#include <condition_variable>

namespace csapex
{

class Connection {
    friend class command::AddFulcrum;
    friend class command::MoveFulcrum;
    friend class command::ModifyFulcrum;
    friend class command::DeleteFulcrum;
    friend class GraphIO;
    friend class Graph;
    friend class Fulcrum;

public:
    typedef std::shared_ptr<Connection> Ptr;

    enum class State {
        NOT_INITIALIZED,
        READY_TO_RECEIVE,
        UNREAD,
        READ,
        DONE
    };

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

    ~Connection();

    Connectable* from() const;
    Connectable* to() const;
    int id() const;

    bool contains(Connectable* c) const;

    void setMessage(const ConnectionTypeConstPtr& msg);
    void notifyMessageSet();

    ConnectionTypeConstPtr getMessage() const;
    void allowNewMessage();

    bool isSourceEnabled() const;
    bool isSinkEnabled() const;

    void establishSource();
    void establishSink();
    void establish();
    bool isSourceEstablished() const;
    bool isSinkEstablished() const;
    bool isEstablished() const;

    void fadeSource();
    void fadeSink();
    bool isFaded() const;

    State getState() const;
    void setState(State s);

    void reset();

    bool inLevel() const;
    bool upLevel() const;
    bool downLevel() const;

public:
    boost::signals2::signal<void()> new_message;
    boost::signals2::signal<void()> endpoint_established;
    boost::signals2::signal<void()> connection_established;

    boost::signals2::signal<void()> deleted;

    boost::signals2::signal<void(bool)> source_enable_changed;
    boost::signals2::signal<void(bool)> sink_enabled_changed;

    boost::signals2::signal<void(Fulcrum*)> fulcrum_added;
    boost::signals2::signal<void(Fulcrum*,bool dropped)> fulcrum_moved;
    boost::signals2::signal<void(Fulcrum*,bool dropped, int which)> fulcrum_moved_handle;
    boost::signals2::signal<void(Fulcrum*,int type)> fulcrum_type_changed;
    boost::signals2::signal<void(Fulcrum*)> fulcrum_deleted;

public:
    bool operator == (const Connection& c) const;

    std::vector<FulcrumPtr> getFulcrums() const;
    int getFulcrumCount() const;
    FulcrumPtr getFulcrum(int fulcrum_id);

private:
    /// COMMANDS
    void addFulcrum(int fulcrum_id, const Point& pos, int type, const Point& handle_in=Point(-10.0, 0.0), const Point& handle_out=Point(10.0, 0.0));
    void modifyFulcrum(int fulcrum_id, int type, const Point& handle_in=Point(-10.0, 0.0), const Point& handle_out=Point(10.0, 0.0));
    void moveFulcrum(int fulcrum_id, const Point &pos, bool dropped);
    void deleteFulcrum(int fulcrum_id);

protected:
    Connectable* from_;
    Connectable* to_;
    int id_;
    bool is_dynamic_;

    bool source_established_;
    bool sink_established_;
    bool established_;

    std::vector<FulcrumPtr> fulcrums_;

    State state_;
    ConnectionTypeConstPtr message_;

    static int next_connection_id_;

    mutable std::recursive_mutex sync;
};

}

#endif // CONNECTION_H
