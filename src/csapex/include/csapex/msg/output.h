#ifndef CONNECTOR_OUT_H
#define CONNECTOR_OUT_H

/// COMPONENT
#include <csapex/command/command_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/model/connectable.h>
#include <csapex/msg/token_traits.h>
#include <csapex/utility/shared_ptr_tools.hpp>

namespace csapex
{

class CSAPEX_EXPORT Output : public Connectable
{
    friend class Graph;
    friend class DesignerIO;

public:
    enum class State {
        ACTIVE,
        IDLE
    };

public:
    Output(const UUID &uuid, ConnectableOwnerWeakPtr owner = ConnectableOwnerWeakPtr());
    ~Output();

    void setOutputTransition(OutputTransition* ot);
    void removeOutputTransition();

    void notifyMessageProcessed();
    void notifyMessageProcessed(Connection *connection);

    virtual bool canOutput() const override
    {
        return true;
    }
    virtual bool isOutput() const override
    {
        return true;
    }
    virtual ConnectorType getConnectorType() const override
    {
        return ConnectorType::OUTPUT;
    }

    void activate();

    State getState() const;
    void setState(State s);

    virtual void disable() override;

    virtual void addMessage(TokenPtr message) = 0;

    virtual bool canReceiveToken() const;
    virtual bool canSendMessages() const;
    virtual bool commitMessages(bool is_activated) = 0;
    virtual void publish();
    virtual void nextMessage() = 0;
    virtual TokenPtr getToken() const = 0;
    virtual TokenPtr getAddedToken() = 0;

    virtual bool targetsCanBeMovedTo(Connectable *other_side) const override;
    virtual bool isConnected() const override;

    virtual void connectionMovePreview(Connectable* other_side) override;
    virtual void validateConnections() override;

    std::vector<ConnectionPtr> getConnections() const;

    virtual bool hasMessage() = 0;
    virtual bool hasMarkerMessage() = 0;

    virtual void reset() override;
    virtual void clearBuffer() = 0;

    virtual bool isConnectionPossible(Connectable* other_side) override;
    virtual void removeConnection(Connectable* other_side) override;
    virtual void removeAllConnectionsNotUndoable() override;

public:
    csapex::slim_signal::Signal<void(Connectable*)> messageSent;


protected:
    OutputTransition* transition_;

    State state_;
};

}

#endif // FOR_OUT_H
