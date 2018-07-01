#ifndef CONNECTABLE_H
#define CONNECTABLE_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/model/token_data.h>
#include <csapex/model/connector.h>
#include <csapex_core/csapex_core_export.h>
#include <csapex/utility/slim_signal.hpp>

/// SYSTEM
#include <mutex>
#include <vector>
#include <atomic>
#include <memory>

namespace csapex
{
class CSAPEX_CORE_EXPORT Connectable : public Connector, public std::enable_shared_from_this<Connectable>
{
    friend class Graph;
    friend class Connection;

public:
    virtual ~Connectable();

    int getCount() const override;

    virtual bool isOutput() const override
    {
        return false;
    }
    virtual bool isInput() const override
    {
        return false;
    }
    virtual bool isOptional() const
    {
        return false;
    }

    bool isVirtual() const;
    void setVirtual(bool _virtual);

    bool isVariadic() const;
    void setVariadic(bool variadic);

    bool isParameter() const;
    void setParameter(bool parameter);

    bool isGraphPort() const;
    void setGraphPort(bool graph);

    bool isEssential() const;
    void setEssential(bool essential);

    virtual void addConnection(ConnectionPtr connection);
    virtual void removeConnection(Connectable* other_side);
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

    int countConnections() const override;
    virtual int maxConnectionCount() const override;

    virtual std::vector<ConnectionPtr> getConnections() const;

    virtual ConnectorDescription getDescription() const override;

    bool hasEnabledConnection() const;
    bool hasActiveConnection() const;

    virtual bool isConnected() const;
    virtual bool isConnectedTo(const UUID& other) const override;
    virtual bool isActivelyConnectedTo(const UUID& other) const override;

    virtual std::vector<UUID> getConnectedPorts() const override;

    virtual void disable();
    virtual void enable();

    virtual void reset();
    virtual void stop();

    virtual void notifyMessageProcessed();

    virtual std::string makeStatusString() const override;

protected:
    void setProcessing(bool processing);
    bool isProcessing() const;

public:
    slim_signal::Signal<void(ConnectablePtr)> connectionStart;
    slim_signal::Signal<void(ConnectablePtr)> disconnected;

    slim_signal::Signal<void(ConnectablePtr)> connection_added_to;
    slim_signal::Signal<void(ConnectablePtr)> connection_removed_to;

    slim_signal::Signal<void(ConnectionPtr)> connection_added;
    slim_signal::Signal<void(ConnectionPtr)> connection_faded;

    slim_signal::Signal<void(ConnectablePtr)> message_processed;
    slim_signal::Signal<void(bool)> connectionEnabled;

protected:
    virtual void removeAllConnectionsNotUndoable() = 0;

protected:
    Connectable(const UUID& uuid, ConnectableOwnerWeakPtr owner);

    void init();

protected:
    mutable std::recursive_mutex io_mutex;
    mutable std::recursive_mutex sync_mutex;

    std::string label_;

    TokenData::ConstPtr type_;
    std::vector<ConnectionPtr> connections_;

    std::atomic<int> count_;
    std::atomic<int> seq_no_;

    bool virtual_;
    bool parameter_;
    bool variadic_;
    bool graph_port_;
    bool essential_;

private:
    std::atomic<bool> enabled_;
    std::atomic<bool> processing_;
};

}  // namespace csapex

#endif  // CONNECTABLE_H
