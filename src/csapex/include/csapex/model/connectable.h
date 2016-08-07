#ifndef CONNECTOR_H
#define CONNECTOR_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/model/token_data.h>
#include <csapex/model/unique.h>
#include <csapex/model/error_state.h>
#include <csapex/model/connector_type.h>

/// SYSTEM
#include <mutex>
#include <vector>
#include <atomic>
#include <csapex/utility/slim_signal.hpp>
#include <memory>

namespace csapex
{

class Connectable : public ErrorState, public Unique, public std::enable_shared_from_this<Connectable>
{
    friend class Graph;
    friend class Connection;

public:
    static const std::string MIME_CREATE_CONNECTION;
    static const std::string MIME_MOVE_CONNECTIONS;

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

    virtual bool isVirtual() const {
        return false;
    }

    virtual void addConnection(ConnectionPtr connection);
    virtual void fadeConnection(ConnectionPtr connection);

    void setLabel(const std::string& label);
    std::string getLabel() const;

    void setType(TokenData::ConstPtr type);
    TokenData::ConstPtr getType() const;

    virtual ConnectorType getConnectorType() const = 0;

    bool isEnabled() const;
    void setEnabled(bool enabled);

    int sequenceNumber() const;
    void setSequenceNumber(int seq_no_);

    std::vector<ConnectionPtr> getConnections() const;

    bool hasActiveConnection() const;

    virtual bool isConnected() const;

    virtual void disable();
    virtual void enable();

    virtual void reset();
    virtual void stop();

    virtual void notifyMessageProcessed();

    /*REFACTOR*/ virtual bool shouldMove(bool left, bool right);
    /*REFACTOR*/ virtual bool shouldCreate(bool left, bool right);

public:
    csapex::slim_signal::Signal<void(bool)> enabled_changed;

    csapex::slim_signal::Signal<void(Connectable*)> disconnected;
    csapex::slim_signal::Signal<void(Connectable*)> connectionStart;
    csapex::slim_signal::Signal<void(Connectable*,Connectable*)> connectionInProgress;

    csapex::slim_signal::Signal<void(Connectable*)> connection_added_to;
    csapex::slim_signal::Signal<void(Connectable*)> connection_removed_to;

    csapex::slim_signal::Signal<void(ConnectionPtr)> connection_added;
    csapex::slim_signal::Signal<void(ConnectionPtr)> connection_faded;

    csapex::slim_signal::Signal<void(bool)> connectionEnabled;
    csapex::slim_signal::Signal<void(Connectable*)> messageProcessed;
    csapex::slim_signal::Signal<void(bool, std::string, int)> connectableError;

    csapex::slim_signal::Signal<void()> typeChanged;
    csapex::slim_signal::Signal<void(std::string)> labelChanged;

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

    void init();

    void errorEvent(bool error, const std::string &msg, ErrorLevel level) override;


protected:
    mutable std::recursive_mutex io_mutex;
    mutable std::recursive_mutex sync_mutex;

    std::string label_;

    TokenData::ConstPtr type_;
    std::vector<ConnectionPtr> connections_;

    int count_;
    int seq_no_;

private:
    std::atomic<bool> enabled_;
};

}

#endif // CONNECTOR_H
