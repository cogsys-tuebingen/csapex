#ifndef CONNECTOR_OUT_H
#define CONNECTOR_OUT_H

/// COMPONENT
#include <csapex/command/command_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/model/connectable.h>
#include <csapex/msg/token_traits.h>
#include <csapex/msg/message_allocator.h>
#include <csapex/utility/shared_ptr_tools.hpp>
#include <csapex/model/observer.h>

namespace csapex
{
class CSAPEX_CORE_EXPORT Output : public Connectable, public MessageAllocator, public Observer
{
    friend class Graph;
    friend class DesignerIO;

public:
    enum class State
    {
        ACTIVE,
        IDLE
    };

public:
    Output(const UUID& uuid, ConnectableOwnerWeakPtr owner = ConnectableOwnerWeakPtr());
    ~Output() override;

    void setOutputTransition(OutputTransition* ot);
    void removeOutputTransition();

    void addConnection(ConnectionPtr connection) override;
    void removeConnection(Connectable* other_side) override;

    void notifyMessageProcessed() override;
    void notifyMessageProcessed(Connection* connection);

    bool isOutput() const override
    {
        return true;
    }

    ConnectorType getConnectorType() const override
    {
        return ConnectorType::OUTPUT;
    }

    void activate();

    State getState() const;
    void setState(State s);

    void enable() override;
    void disable() override;

    virtual void addMessage(TokenPtr message) = 0;

    virtual bool canReceiveToken() const;
    virtual bool canSendMessages() const;
    virtual bool commitMessages(bool is_activated) = 0;
    virtual void publish();
    virtual void republish();
    virtual TokenPtr getToken() const = 0;
    virtual TokenPtr getAddedToken() = 0;

    bool isConnected() const override;

    std::vector<ConnectionPtr> getConnections() const override;

    virtual bool hasMessage() = 0;
    virtual bool hasMarkerMessage() = 0;

    void reset() override;
    virtual void clearBuffer() = 0;

    void removeAllConnectionsNotUndoable() override;

public:
    slim_signal::Signal<void(Connectable*)> messageSent;

protected:
    void addStatusInformation(std::stringstream& status_stream) const override;

protected:
    OutputTransition* transition_;

    State state_;
};

}  // namespace csapex

#endif  // FOR_OUT_H
