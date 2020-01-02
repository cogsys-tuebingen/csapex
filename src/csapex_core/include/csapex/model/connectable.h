#ifndef CONNECTABLE_H
#define CONNECTABLE_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/model/token_data.h>
#include <csapex/model/connector.h>
#include <csapex_core/csapex_core_export.h>
#include <csapex/utility/slim_signal.hpp>
#include <csapex/utility/thread_debug_helper.hpp>

/// SYSTEM
#include <mutex>
#include <vector>
#include <atomic>
#include <memory>

namespace csapex
{
class CSAPEX_CORE_EXPORT Connectable : public Connector, public std::enable_shared_from_this<Connectable>, private ThreadDebugHelper
{
    friend class Graph;
    friend class Connection;

public:
    ~Connectable() override;

    int getCount() const override;

    bool isOutput() const override
    {
        return false;
    }
    bool isInput() const override
    {
        return false;
    }
    bool isOptional() const override
    {
        return false;
    }

    bool isVirtual() const override;
    void setVirtual(bool _virtual);

    bool isVariadic() const;
    void setVariadic(bool variadic);

    bool isParameter() const override;
    void setParameter(bool parameter);

    bool isGraphPort() const override;
    void setGraphPort(bool graph);

    bool isEssential() const override;
    void setEssential(bool essential);

    virtual void addConnection(ConnectionPtr connection);
    virtual void removeConnection(Connectable* other_side);
    virtual void fadeConnection(ConnectionPtr connection);

    void setLabel(const std::string& label);
    std::string getLabel() const override;

    void setType(TokenData::ConstPtr type);
    TokenData::ConstPtr getType() const override;

    ConnectorType getConnectorType() const override = 0;

    bool isEnabled() const override;
    void setEnabled(bool enabled);

    int sequenceNumber() const override;
    void setSequenceNumber(int seq_no_);

    int countConnections() const override;
    int maxConnectionCount() const override;

    virtual std::vector<ConnectionPtr> getConnections() const;

    ConnectorDescription getDescription() const override;

    bool hasEnabledConnection() const;
    bool hasActiveConnection() const override;

    bool isConnected() const override;
    bool isConnectedTo(const UUID& other) const override;
    bool isActivelyConnectedTo(const UUID& other) const override;

    std::vector<UUID> getConnectedPorts() const override;

    virtual void disable();
    virtual void enable();

    virtual void reset();
    virtual void stop();

    virtual void notifyMessageProcessed();

    std::string makeStatusString() const override;

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

    mutable std::recursive_mutex processing_mutex_;

private:
    std::atomic<bool> enabled_;
    bool processing_;
};

}  // namespace csapex

#endif  // CONNECTABLE_H
