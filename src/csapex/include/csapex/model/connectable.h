#ifndef CONNECTOR_H
#define CONNECTOR_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/model/connection_type.h>
#include <csapex/model/unique.h>
#include <csapex/model/error_state.h>

/// SYSTEM
#include <mutex>
#include <atomic>

/// FORWARDS DECLARATION
namespace csapex
{

class Connectable : public ErrorState, public Unique
{
    friend class Port;
    friend class Graph;

public:
    static const std::string MIME_CREATE_CONNECTION;
    static const std::string MIME_MOVE_CONNECTIONS;

    static UUID makeUUID(const UUID &box_uuid, const std::string &type, int sub_id);

public:
    virtual ~Connectable();

    int getCount() const;

    virtual bool canConnectTo(Connectable* other_side, bool move) const;

    virtual bool canOutput() const {
        return false;
    }
    virtual bool canInput() const {
        return false;
    }
    virtual bool isOutput() const {
        return false;
    }
    virtual bool isInput() const {
        return false;
    }

    bool isDynamic() const;

    void setLabel(const std::string& label);
    std::string getLabel() const;

    void setType(ConnectionType::ConstPtr type);
    ConnectionType::ConstPtr getType() const;

    void setLevel(int level);
    int getLevel() const;

    bool isEnabled() const;
    void setEnabled(bool enabled);

    int sequenceNumber() const;
    void setSequenceNumber(int seq_no_);

    std::vector<ConnectionPtr> getConnections() const;
    virtual void addConnection(ConnectionPtr connection);
    virtual void fadeConnection(ConnectionPtr connection);

    virtual bool isConnected() const;

    virtual void disable();
    virtual void enable();

    virtual void stop();

    virtual void notifyMessageProcessed();

public:
    boost::signals2::signal<void(bool)> enabled_changed;

    boost::signals2::signal<void(Connectable*)> disconnected;
    boost::signals2::signal<void(Connectable*)> connectionStart;
    boost::signals2::signal<void(Connectable*,Connectable*)> connectionInProgress;
    boost::signals2::signal<void(Connectable*)> connectionDone;
    boost::signals2::signal<void(Connectable*)> connectionRemoved;
    boost::signals2::signal<void(bool)> connectionEnabled;
    boost::signals2::signal<void(Connectable*)> messageProcessed;
    boost::signals2::signal<void(bool, std::string, int)> connectableError;

    boost::signals2::signal<void(Connectable*)> messageSent;
    boost::signals2::signal<void(Connectable*)> messageArrived;
    boost::signals2::signal<void()> typeChanged;

public:
    /**
     * INTERFACE
     */
    virtual bool targetsCanBeMovedTo(Connectable* other_side) const = 0;
    virtual bool isConnectionPossible(Connectable* other_side) = 0;
    virtual void removeConnection(Connectable* other_side) = 0;
    virtual void validateConnections();
    virtual void connectionMovePreview(Connectable* other_side) = 0;

protected:
    virtual void removeAllConnectionsNotUndoable() = 0;

protected:
    Connectable(const UUID &uuid);
    Connectable(Unique *parent, int sub_id, const std::string &type);

    void setDynamic(bool dynamic);

    void init();

    void errorEvent(bool error, const std::string &msg, ErrorLevel level);


protected:
    virtual bool shouldMove(bool left, bool right);
    virtual bool shouldCreate(bool left, bool right);

protected:
    mutable std::recursive_mutex io_mutex;
    mutable std::recursive_mutex sync_mutex;

    std::string label_;

    ConnectionType::ConstPtr type_;
    std::vector<ConnectionPtr> connections_;

    int count_;
    int seq_no_;

private:
    std::atomic<bool> enabled_;
    std::atomic<bool> dynamic_;
    int level_;
};

}

#endif // CONNECTOR_H
