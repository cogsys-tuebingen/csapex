#ifndef INPUT_H
#define INPUT_H

/// COMPONENT
#include <csapex/msg/msg_fwd.h>
#include <csapex/model/connectable.h>
#include <csapex/msg/message.h>

namespace csapex
{

class CSAPEX_EXPORT Input : public Connectable
{
    friend class Output;

public:
    Input(const UUID &uuid, ConnectableOwnerWeakPtr owner = ConnectableOwnerWeakPtr());
    virtual ~Input();

    void setInputTransition(InputTransition* it);
    void removeInputTransition();

    virtual bool canInput() const override {
        return true;
    }
    virtual bool isInput() const override {
        return true;
    }
    virtual ConnectorType getConnectorType() const override
    {
        return ConnectorType::INPUT;
    }

    bool canConnectTo(Connectable* other_side, bool move) const override;

    virtual void setToken(TokenPtr message);
    virtual TokenPtr getToken() const;

    virtual bool targetsCanBeMovedTo(Connectable* other_side) const override;

    virtual void connectionMovePreview(Connectable* other_side) override;
    virtual void validateConnections() override;

    OutputPtr getSource() const;

    virtual void removeAllConnectionsNotUndoable() override;

    bool isOptional() const;
    void setOptional(bool optional);

    virtual bool hasMessage() const;
    virtual bool hasReceived() const;

    void free();
    void stop() override;

    virtual void enable() override;
    virtual void disable() override;

    virtual void notifyMessageAvailable(Connection *connection);

    virtual void reset() override;

public:
    slim_signal::Signal<void(Connectable*)> message_set;
    slim_signal::Signal<void(Connection* )> message_available;

protected:
    virtual bool isConnectionPossible(Connectable* other_side) override;
    virtual void removeConnection(Connectable* other_side) override;

protected:
    InputTransition* transition_;

    mutable std::mutex message_mutex_;
    TokenPtr message_;

    bool optional_;
};

}

#endif // INPUT_H
