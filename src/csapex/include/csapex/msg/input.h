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
    virtual ConnectionTypeConstPtr getMessage() const;

    virtual bool targetsCanBeMovedTo(Connectable* other_side) const;

    virtual void connectionMovePreview(Connectable* other_side);
    virtual void validateConnections();

    Connectable* getSource() const;

    virtual void removeAllConnectionsNotUndoable();

    bool isOptional() const;
    void setOptional(bool optional);

    virtual bool hasMessage() const;
    virtual bool hasReceived() const;

    void free();
    void stop();

    virtual void enable();
    virtual void disable();

    virtual void notifyMessageProcessed();

    void reset();


protected:
    virtual bool isConnectionPossible(Connectable* other_side);
    virtual void removeConnection(Connectable* other_side);

protected:
    mutable std::mutex message_mutex_;
    ConnectionTypeConstPtr message_;

    bool optional_;
};

}

#endif // INPUT_H
