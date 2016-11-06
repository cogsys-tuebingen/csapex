#ifndef CONNECTION_H
#define CONNECTION_H

/// COMPONENT
#include <csapex/data/point.h>
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/utility/slim_signal.hpp>
#include <csapex/model/token.h>
#include <csapex/csapex_export.h>

/// SYSTEM
#include <memory>
#include <vector>
#include <deque>
#include <mutex>
#include <condition_variable>

namespace csapex
{

class CSAPEX_EXPORT Connection
{
    friend class GraphIO;
    friend class Graph;
    friend class Fulcrum;

public:
    typedef std::shared_ptr<Connection> Ptr;

    enum class State {
        NOT_INITIALIZED,
        UNREAD,
        READ,
        DONE = NOT_INITIALIZED
    };

public:
    friend std::ostream& operator << (std::ostream& out, const Connection& c);

protected:
    Connection(OutputPtr from, InputPtr to);
    Connection(OutputPtr from, InputPtr to, int id);

public:
    virtual ~Connection();

    void detach(Connectable* c);
    bool isDetached() const;

    OutputPtr from() const;
    InputPtr to() const;
    int id() const;

    bool contains(Connectable* c) const;

    virtual void setToken(const TokenPtr &msg);

    TokenPtr getToken() const;
    void setTokenProcessed();

    /**
     * @brief readMessage retrieves the current message and marks the Connection read
     * @return
     */
    TokenPtr readToken();

    bool holdsActiveToken() const;

    bool isActive() const;
    void setActive(bool active);

    bool isEnabled() const;
    bool isSourceEnabled() const;
    bool isSinkEnabled() const;

    State getState() const;
    void setState(State s);

    void reset();

public:
    csapex::slim_signal::Signal<void()> deleted;

    csapex::slim_signal::Signal<void(bool)> source_enable_changed;
    csapex::slim_signal::Signal<void(bool)> sink_enabled_changed;

    csapex::slim_signal::Signal<void(Fulcrum*)> fulcrum_added;
    csapex::slim_signal::Signal<void(Fulcrum*,bool dropped)> fulcrum_moved;
    csapex::slim_signal::Signal<void(Fulcrum*,bool dropped, int which)> fulcrum_moved_handle;
    csapex::slim_signal::Signal<void(Fulcrum*,int type)> fulcrum_type_changed;
    csapex::slim_signal::Signal<void(Fulcrum*)> fulcrum_deleted;

public:
    bool operator == (const Connection& c) const;

    std::vector<FulcrumPtr> getFulcrums() const;
    int getFulcrumCount() const;
    FulcrumPtr getFulcrum(int fulcrum_id);

    void addFulcrum(int fulcrum_id, const Point& pos, int type, const Point& handle_in=Point(-10.0, 0.0), const Point& handle_out=Point(10.0, 0.0));
    void modifyFulcrum(int fulcrum_id, int type, const Point& handle_in=Point(-10.0, 0.0), const Point& handle_out=Point(10.0, 0.0));
    void moveFulcrum(int fulcrum_id, const Point &pos, bool dropped);
    void deleteFulcrum(int fulcrum_id);

protected:
    void notifyMessageSet();
    void notifyMessageProcessed();

protected:
    OutputPtr from_;
    InputPtr to_;
    int id_;

    bool active_;

    bool detached_;

    std::vector<FulcrumPtr> fulcrums_;

    State state_;
    TokenPtr message_;

    static int next_connection_id_;

    mutable std::recursive_mutex sync;
};

}

#endif // CONNECTION_H
