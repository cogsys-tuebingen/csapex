#ifndef CONNECTOR_H
#define CONNECTOR_H

/// COMPONENT
#include <csapex/model/connection_type.h>
#include <csapex/model/unique.h>
#include <csapex/command/command.h>
#include <csapex/model/error_state.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QMutex>
#include <QWaitCondition>
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

    static const std::string namespace_separator;

    enum {
        TYPE_IN = 1,
        TYPE_OUT = 0,
        TYPE_MISC = -1
    };

    static UUID makeUUID(const UUID &box_uuid, int type, int sub_id);

public:
    void setPort(Port* port);
    Port* getPort() const;

    CommandDispatcher* getCommandDispatcher() const;
    void setCommandDispatcher(CommandDispatcher* d);

    GraphPtr getGraph() const;

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

    virtual bool isForwarding() const;

    void setLabel(const std::string& label);
    std::string getLabel() const;

    void setType(ConnectionType::ConstPtr type);
    ConnectionType::ConstPtr getType() const;

    void setMinimizedSize(bool mini);
    bool isMinimizedSize() const;

    bool isAsync() const;
    void setAsync(bool asynch);
    void setTempAsync(bool asynch);

    bool isBlocked() const;

    /**
     * INTERFACE
     */
    virtual bool targetsCanBeMovedTo(Connectable* other_side) const = 0;
    virtual bool isConnected() const = 0;
    virtual bool tryConnect(Connectable* other_side) = 0;
    virtual void removeConnection(Connectable* other_side) = 0;
    virtual void validateConnections();
    virtual void connectionMovePreview(Connectable* other_side) = 0;
    virtual Command::Ptr removeAllConnectionsCmd() = 0;
protected:
    virtual void removeAllConnectionsNotUndoable() = 0;

public Q_SLOTS:
    virtual bool tryConnect(QObject* other_side);
    virtual void removeConnection(QObject* other_side);

    void removeAllConnectionsUndoable();

    virtual void disable();
    virtual void enable();

    bool isEnabled() const;

    virtual void setProcessing(bool processing);
    bool isProcessing() const;
    virtual void waitForProcessing(const UUID &who_is_waiting);
    virtual void updateIsProcessing();

    void stop();

protected:
    virtual void notifyMessageProcessed();

    void setBlocked(bool b);


Q_SIGNALS:
    void disconnected(QObject*);
    void enabled(bool e);
    void connectionStart();
    void connectionInProgress(Connectable*, Connectable*);
    void connectionDone();
    void connectionRemoved();
    void messageProcessed();

Q_SIGNALS:
    void messageSent(Connectable* source);
    void messageArrived(Connectable* source);
    void typeChanged();

protected:
    Connectable(const UUID &uuid);
    Connectable(Unique *parent, int sub_id, int type);
    virtual ~Connectable();
    void init();

    void errorEvent(bool error, const std::string &msg, ErrorLevel level);


protected:
    virtual bool shouldMove(bool left, bool right);
    virtual bool shouldCreate(bool left, bool right);

protected:
    CommandDispatcher* dispatcher_;
    Port* port_;

    mutable QMutex io_mutex;

    QWaitCondition can_process_cond;

    csapex::DesignBoard* designer;

    Qt::MouseButtons buttons_down_;

    std::string label_;

    ConnectionType::ConstPtr type_;

    bool minimized_;
    int count_;

public:
    static bool allow_processing;

private:
    bool processing;
    bool enabled_;
    bool async_;
    bool async_temp_;
    bool blocked_;

public:
    std::vector<UUID> waiting_list_;
};

}

#endif // CONNECTOR_H
