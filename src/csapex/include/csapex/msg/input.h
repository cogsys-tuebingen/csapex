#ifndef CONNECTOR_IN_H
#define CONNECTOR_IN_H

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/csapex_fwd.h>
#include <csapex/msg/message.h>
#include <csapex/utility/buffer.h>

namespace csapex
{

class Input : public Connectable
{
    Q_OBJECT

    friend class Output;
    friend class command::AddConnection;
    friend class command::MoveConnection;
    friend class command::DeleteConnection;

public:
    Input(const UUID &uuid);
    Input(Unique *parent, int sub_id);
    virtual ~Input();

    virtual bool canInput() const {
        return true;
    }
    virtual bool isInput() const {
        return true;
    }

    bool canConnectTo(Connectable* other_side, bool move) const;

    void inputMessage(ConnectionType::ConstPtr message);
    ConnectionTypeConstPtr getMessage() const;

    virtual bool targetsCanBeMovedTo(Connectable* other_side) const;
    virtual bool isConnected() const;

    virtual void connectionMovePreview(Connectable* other_side);
    virtual void validateConnections();

    Connectable* getSource() const;

    virtual CommandPtr removeAllConnectionsCmd();
    virtual void removeAllConnectionsNotUndoable();

    bool isOptional() const;
    void setOptional(bool optional);

    bool hasMessage() const;
    bool hasReceived() const;

    void free();
    void stop();

    virtual void enable();
    virtual void disable();

    virtual void notifyMessageProcessed();

    void reset();

protected:
    virtual bool tryConnect(Connectable* other_side);
    virtual bool acknowledgeConnection(Connectable* other_side);
    virtual void removeConnection(Connectable* other_side);

Q_SIGNALS:
    void gotMessage(ConnectionType::ConstPtr msg);

private Q_SLOTS:
    void handleMessage(ConnectionType::ConstPtr msg);

protected:
    Connectable* target;

    BufferPtr buffer_;

    bool optional_;
};

}

#endif // CONNECTOR_IN_H
