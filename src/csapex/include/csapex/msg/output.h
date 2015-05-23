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
    enum class State {
        RECEIVING, // output is collecting messages
        ACTIVE,    // output has sent it's messages, no answer yet
        IDLE       // output's messages have all been processed
    };

public:
    Output(OutputTransition* transition, const UUID &uuid);
    Output(OutputTransition* transition, Unique *parent, int sub_id);
    ~Output();

    OutputTransition* getTransition() const;

    virtual bool canOutput() const override
    {
        return true;
    }
    virtual bool isOutput() const override
    {
        return true;
    }

    State getState() const;

    virtual void addConnection(ConnectionPtr connection) override;
    void fadeConnection(ConnectionPtr connection) override;

    virtual void disable() override;

    virtual void setMultipart(bool multipart, bool last_part) = 0;
    virtual void publish(ConnectionType::ConstPtr message) = 0;

    virtual bool canSendMessages() const;
    virtual void commitMessages() = 0;
    virtual void nextMessage() = 0;
    virtual ConnectionTypeConstPtr getMessage() const = 0;

    virtual bool targetsCanBeMovedTo(Connectable *other_side) const override;
    virtual bool isConnected() const override;

    virtual bool isForced() const;

    virtual void connectionMovePreview(Connectable* other_side) override;
    virtual void validateConnections() override;

    int countConnections();
    std::vector<ConnectionPtr> getConnections() const;

    CommandPtr removeAllConnectionsCmd() override;
    CommandPtr removeConnectionCmd(Connection *connection);

    void forceSendMessage(bool force = true);

    virtual bool hasMessage() = 0;

    virtual void reset();
    virtual void clear() = 0;

protected:
    void setState(State s);

protected:
    /// PRIVATE: Use command to create a connection (undoable)
    virtual bool isConnectionPossible(Connectable* other_side) override;
    virtual void removeConnection(Connectable* other_side) override;
    virtual void removeAllConnectionsNotUndoable() override;

protected:
    OutputTransition* transition_;

    bool force_send_message_;
    State state_;
};

}

#endif // FOR_OUT_H
