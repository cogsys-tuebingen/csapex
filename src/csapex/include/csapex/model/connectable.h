#ifndef CONNECTOR_H
#define CONNECTOR_H

/// COMPONENT
#include <csapex/model/connection_type.h>
#include <csapex/model/unique.h>
#include <csapex/model/error_state.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <mutex>
#include <QObject>

/// FORWARDS DECLARATION
namespace csapex
{

class Connectable : public QObject, public ErrorState, public Unique
{
    Q_OBJECT

    friend class Port;
    friend class Graph;

public:
    static const QString MIME_CREATE_CONNECTION;
    static const QString MIME_MOVE_CONNECTIONS;

    static UUID makeUUID(const UUID &box_uuid, const std::string &type, int sub_id);

public:
    int getCount() const;

    virtual bool canConnectTo(Connectable* other_side, bool move) const;

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

    void setDynamic(bool dynamic);
    bool isDynamic() const;

    void setLabel(const std::string& label);
    std::string getLabel() const;

    void setType(ConnectionType::ConstPtr type);
    ConnectionType::ConstPtr getType() const;

    bool isEnabled() const;
    void setEnabled(bool enabled);

    int sequenceNumber() const;
    void setSequenceNumber(int seq_no_);

    std::vector<ConnectionWeakPtr> getConnections() const;
    virtual void addConnection(ConnectionWeakPtr connection);
    virtual void removeConnection(ConnectionWeakPtr connection);

    virtual bool isConnected() const;

    /**
     * INTERFACE
     */
    virtual bool targetsCanBeMovedTo(Connectable* other_side) const = 0;
    virtual bool isConnectionPossible(Connectable* other_side) = 0;
    virtual void removeConnection(Connectable* other_side) = 0;
    virtual void validateConnections();
    virtual void connectionMovePreview(Connectable* other_side) = 0;
    virtual CommandPtr removeAllConnectionsCmd() = 0;
protected:
    virtual void removeAllConnectionsNotUndoable() = 0;

public Q_SLOTS:
    virtual bool isConnectionPossible(QObject* other_side);
    virtual void removeConnection(QObject* other_side);

    virtual void disable();
    virtual void enable();

    virtual void stop();

    virtual void notifyMessageProcessed();


Q_SIGNALS:
    void disconnected(QObject*);
    void enabled(bool e);
    void connectionStart(Connectable*);
    void connectionInProgress(Connectable*, Connectable*);
    void connectionDone(Connectable*);
    void connectionRemoved(Connectable*);
    void connectionEnabled(bool);
    void messageProcessed(Connectable*);
    void connectableError(bool error, const std::string &msg, int level);

    void messageSent(Connectable* source);
    void messageArrived(Connectable* source);
    void typeChanged();

protected:
    Connectable(const UUID &uuid);
    Connectable(Unique *parent, int sub_id, const std::string &type);
    virtual ~Connectable();
    void init();

    void errorEvent(bool error, const std::string &msg, ErrorLevel level);


protected:
    virtual bool shouldMove(bool left, bool right);
    virtual bool shouldCreate(bool left, bool right);

protected:
    mutable std::recursive_mutex io_mutex;
    mutable std::recursive_mutex sync_mutex;

    csapex::DesignBoard* designer;

    Qt::MouseButtons buttons_down_;

    std::string label_;

    ConnectionType::ConstPtr type_;
    std::vector<ConnectionWeakPtr> connections_;

    int count_;
    int seq_no_;

private:
    bool enabled_;
    bool dynamic_;
};

}

#endif // CONNECTOR_H
