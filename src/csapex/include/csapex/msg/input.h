#ifndef INPUT_H
#define INPUT_H

/// COMPONENT
#include <csapex/msg/msg_fwd.h>
#include <csapex/model/connectable.h>
#include <csapex/msg/message.h>

namespace csapex
{

class Input : public Connectable
{
    friend class Output;

public:
    Input(const UUID &uuid);
    virtual ~Input();

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

    virtual void inputMessage(Token::ConstPtr message);
    virtual TokenConstPtr getMessage() const;

    virtual bool targetsCanBeMovedTo(Connectable* other_side) const override;

    virtual void connectionMovePreview(Connectable* other_side) override;
    virtual void validateConnections() override;

    Connectable* getSource() const;

    virtual void removeAllConnectionsNotUndoable() override;

    bool isOptional() const;
    void setOptional(bool optional);

    virtual bool hasMessage() const;
    virtual bool hasReceived() const;

    void free();
    void stop() override;

    virtual void enable() override;
    virtual void disable() override;

    virtual void notifyMessageProcessed() override;

    virtual void reset() override;


protected:
    virtual bool isConnectionPossible(Connectable* other_side) override;
    virtual void removeConnection(Connectable* other_side) override;

protected:
    mutable std::mutex message_mutex_;
    TokenConstPtr message_;

    bool optional_;
};

}

#endif // INPUT_H
