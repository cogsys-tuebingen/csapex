#ifndef TRIGGER_H
#define TRIGGER_H

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/csapex_fwd.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/msg/generic_value_message.hpp>

namespace csapex
{

class Trigger : public Connectable
{
    friend class Input;
    friend class Graph;
    friend class command::AddConnection;
    friend class command::MoveConnection;
    friend class command::DeleteConnection;
    friend class DesignerIO;

public:
    Trigger(Settings& settings, const UUID &uuid);
    Trigger(Settings& settings, Unique *parent, int sub_id);
    ~Trigger();

    virtual bool canOutput() const {
        return true;
    }
    virtual bool isOutput() const {
        return true;
    }

    void trigger();

    virtual void disable();

    virtual bool canConnectTo(Connectable* other_side, bool move) const;

    virtual bool targetsCanBeMovedTo(Connectable *other_side) const;
    virtual bool isConnected() const;

    virtual void connectionMovePreview(Connectable* other_side);
    virtual void validateConnections();

    int noTargets();
    std::vector<Slot*> getTargets() const;

    void connectForcedWithoutCommand(Slot* other_side);

    virtual CommandPtr removeAllConnectionsCmd();
    CommandPtr removeConnectionCmd(Slot *other_side);


    void reset();

protected:
    /// PRIVATE: Use command to create a connection (undoable)
    virtual bool tryConnect(Connectable* other_side);
    virtual void removeConnection(Connectable* other_side);
    virtual void removeAllConnectionsNotUndoable();

    bool connect(Connectable* other_side);

protected:
    std::vector<Slot*> targets_;

};

}

#endif // TRIGGER_H
