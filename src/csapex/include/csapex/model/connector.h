#ifndef CONNECTOR_H
#define CONNECTOR_H

/// COMPONENT
#include <csapex/model/connection_type.h>
#include <csapex/command/command.h>
#include <csapex/model/error_state.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QObject>

/// FORWARDS DECLARATION
namespace csapex
{

class Connectable : public QObject, public ErrorState
{
    Q_OBJECT

    friend class Port;


public:
    static const QString MIME_CREATE_CONNECTION;
    static const QString MIME_MOVE_CONNECTIONS;

    static const std::string namespace_separator;

    enum {
        TYPE_IN = 1,
        TYPE_OUT = 0,
        TYPE_MISC = -1
    };

    static std::string makeUUID(const std::string &box_uuid, int type, int sub_id);

public:
    void setPort(Port* port);
    Port* getPort() const;

    virtual bool canConnectTo(Connectable* other_side, bool move) const;

    virtual bool targetsCanBeMovedTo(Connectable* other_side) const = 0;
    virtual bool isConnected() const = 0;
    virtual bool tryConnect(Connectable* other_side) = 0;
    virtual void removeConnection(Connectable* other_side) = 0;
    virtual void validateConnections();

    virtual void connectionMovePreview(Connectable* other_side) = 0;

    virtual bool canOutput() const {
        return false;
    }
    virtual bool canInput() const {
        return false;
    }
    virtual bool isOutput() const {
        return false;
    }
    virtual bool isInput() const {
        return false;
    }

    virtual bool isForwarding() const;

    std::string UUID() const;

    void setLabel(const std::string& label);
    std::string getLabel() const;

    void setType(ConnectionType::ConstPtr type);
    ConnectionType::ConstPtr getType() const;

    void setMinimizedSize(bool mini);
    bool isMinimizedSize() const;

    virtual Command::Ptr removeAllConnectionsCmd() = 0;

    Node* getNode() const;
    int getCount() const;

public Q_SLOTS:
    virtual bool tryConnect(QObject* other_side);
    virtual void removeConnection(QObject* other_side);

    void removeAllConnectionsUndoable();

    virtual void disable();
    virtual void enable();


Q_SIGNALS:
    void disconnected(QObject*);
    void disabled(Connectable* source);
    void enabled(Connectable* source);
    void connectionStart();
    void connectionInProgress(Connectable*, Connectable*);
    void connectionDone();

Q_SIGNALS:
    void messageSent(Connectable* source);
    void messageArrived(Connectable* source);

protected:
    Connectable(Node* parent, const std::string &uuid);
    Connectable(Node* parent, int sub_id, int type);
    virtual ~Connectable();
    void init();

    virtual void removeAllConnectionsNotUndoable() = 0;

    void errorEvent(bool error, const std::string &msg, ErrorLevel level);
    void errorChanged(bool error);

protected:
    virtual bool shouldMove(bool left, bool right);
    virtual bool shouldCreate(bool left, bool right);

protected:
    Node* node_;
    Port* port_;

    csapex::DesignBoard* designer;

    Qt::MouseButtons buttons_down_;

    std::string uuid_;
    std::string label_;

    ConnectionType::ConstPtr type_;

    bool minimized_;
    int count_;
};

}

#endif // CONNECTOR_H
