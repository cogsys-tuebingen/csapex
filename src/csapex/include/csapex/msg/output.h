#ifndef CONNECTOR_OUT_H
#define CONNECTOR_OUT_H

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/msg/message_traits.h>
#include <csapex/csapex_fwd.h>
#include <csapex/utility/shared_ptr_tools.hpp>

namespace csapex
{

class Output : public Connectable
{
    friend class Input;
    friend class Graph;
    friend class command::AddConnection;
    friend class command::MoveConnection;
    friend class command::DeleteConnection;
    friend class DesignerIO;

public:
    Output(const UUID &uuid);
    Output(Unique *parent, int sub_id);
    ~Output();

    virtual bool canOutput() const override
    {
        return true;
    }
    virtual bool isOutput() const override
    {
        return true;
    }

    virtual void disable() override;

    virtual void publish(ConnectionType::ConstPtr message) = 0;

    virtual bool canSendMessages() const;
    virtual bool sendMessages() = 0;

    virtual bool targetsCanBeMovedTo(Connectable *other_side) const override;
    virtual bool isConnected() const override;

    virtual void connectionMovePreview(Connectable* other_side) override;
    virtual void validateConnections() override;

    int noTargets();
    std::vector<Input*> getTargets() const;

    void connectForcedWithoutCommand(Input* other_side);

    virtual CommandPtr removeAllConnectionsCmd() override;
    virtual CommandPtr removeConnectionCmd(Input *other_side);

    void forceSendMessage(bool force = true);

    virtual bool hasMessage() = 0;

    virtual void reset();
    virtual void clear() = 0;

protected:
    /// PRIVATE: Use command to create a connection (undoable)
    virtual bool tryConnect(Connectable* other_side) override;
    virtual void removeConnection(Connectable* other_side) override;
    virtual void removeAllConnectionsNotUndoable() override;

    bool connect(Connectable* other_side);

protected:
    std::vector<Input*> targets_;
    bool force_send_message_;
};

}

#endif // CONNECTOR_OUT_H
