#ifndef CONNECTOR_H
#define CONNECTOR_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/model/unique.h>
#include <csapex/model/error_state.h>
#include <csapex/model/connector_type.h>
#include <csapex/model/connector_description.h>
#include <csapex_core/csapex_core_export.h>

/// SYSTEM
#include <csapex/utility/slim_signal.hpp>

namespace csapex
{
class CSAPEX_CORE_EXPORT Connector : public Unique
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
    virtual TokenDataConstPtr getType() const = 0;
    virtual ConnectorType getConnectorType() const = 0;
    virtual ConnectorDescription getDescription() const = 0;

    virtual bool isEnabled() const = 0;
    virtual int sequenceNumber() const = 0;
    virtual int countConnections() const = 0;
    virtual int maxConnectionCount() const = 0;

    virtual bool hasActiveConnection() const = 0;
    virtual bool isConnected() const = 0;
    virtual bool isConnectedTo(const UUID& other) const = 0;
    virtual bool isActivelyConnectedTo(const UUID& other) const = 0;

    virtual std::vector<UUID> getConnectedPorts() const = 0;

    // DEBUG INFORMATION
    virtual std::string makeStatusString() const = 0;

public:
    slim_signal::Signal<void(bool)> enabled_changed;
    slim_signal::Signal<void(bool)> essential_changed;

    slim_signal::Signal<void(TokenDataConstPtr)> typeChanged;
    slim_signal::Signal<void(std::string)> labelChanged;

protected:
    Connector(const UUID& uuid, ConnectableOwnerWeakPtr owner);

    virtual void addStatusInformation(std::stringstream& status_stream) const;

protected:
    ConnectableOwnerWeakPtr owner_;
};

}  // namespace csapex
#endif  // CONNECTOR_H
