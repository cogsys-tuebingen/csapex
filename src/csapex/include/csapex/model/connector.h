#ifndef CONNECTOR_H
#define CONNECTOR_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/model/unique.h>
#include <csapex/model/error_state.h>
#include <csapex/model/connector_type.h>
#include <csapex/model/connector_description.h>
#include <csapex/csapex_export.h>

/// SYSTEM
#include <csapex/utility/slim_signal.hpp>

namespace csapex
{

class CSAPEX_EXPORT Connector : public Unique, public ErrorState, public std::enable_shared_from_this<Connector>
{
public:
    ConnectableOwnerPtr getOwner() const;

    virtual int getCount() const = 0;

    virtual bool isOutput() const = 0;
    virtual bool isInput() const = 0;
    virtual bool isOptional() const = 0;
    virtual bool isParameter() const = 0;
    virtual bool isSynchronous() const;
    virtual bool isAsynchronous() const;
    virtual bool isVirtual() const = 0;
    virtual bool isGraphPort() const = 0;
    virtual bool isEssential() const = 0;

    virtual std::string getLabel() const = 0;
    virtual TokenData::ConstPtr getType() const = 0;
    virtual ConnectorType getConnectorType() const = 0;
    virtual ConnectorDescription getDescription() const = 0;

    virtual bool isEnabled() const = 0;
    virtual int sequenceNumber() const = 0;
    virtual int countConnections() const = 0;
    virtual int maxConnectionCount() const = 0;

    virtual bool hasActiveConnection() const = 0;
    virtual bool isConnected() const = 0;
    virtual bool isConnectedTo(const UUID& other) const = 0;

    virtual std::vector<UUID> getConnectedPorts() const = 0;

    // DEBUG INFORMATION
    virtual std::string makeStatusString() const = 0;

public:
    slim_signal::Signal<void(bool)> enabled_changed;

    slim_signal::Signal<void()> essential_changed;

    slim_signal::Signal<void(ConnectorPtr)> disconnected;
    slim_signal::Signal<void(ConnectorPtr)> connectionStart;

    slim_signal::Signal<void(ConnectorPtr)> connection_added_to;
    slim_signal::Signal<void(ConnectorPtr)> connection_removed_to;

    slim_signal::Signal<void(ConnectionPtr)> connection_added;
    slim_signal::Signal<void(ConnectionPtr)> connection_faded;

    slim_signal::Signal<void(bool)> connectionEnabled;
    slim_signal::Signal<void(ConnectorPtr)> message_processed;
    slim_signal::Signal<void(bool, std::string, int)> connectableError;

    slim_signal::Signal<void()> typeChanged;
    slim_signal::Signal<void(std::string)> labelChanged;

protected:
    Connector(const UUID &uuid, ConnectableOwnerWeakPtr owner);

    virtual void addStatusInformation(std::stringstream& status_stream) const;

protected:
    ConnectableOwnerWeakPtr owner_;
};

}
#endif // CONNECTOR_H
