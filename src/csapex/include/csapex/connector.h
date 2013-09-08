#ifndef CONNECTOR_H
#define CONNECTOR_H

/// COMPONENT
#include <csapex/connection_type.h>
#include <csapex/command.h>
#include <csapex/displayable.h>
#include <csapex/csapex_fwd.h>

/// FORWARDS DECLARATION
namespace csapex
{

class Connector : public Displayable
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
    static const QString MIME_CREATE;
    static const QString MIME_MOVE;

    enum {
        TYPE_IN = 1,
        TYPE_OUT = 0,
        TYPE_MISC = -1
    };

    static std::string makeUUID(const std::string &box_uuid, int type, int sub_id);

public:
    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseMoveEvent(QMouseEvent * e);
    virtual void mouseReleaseEvent(QMouseEvent* e);

    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);

    virtual bool canConnect() = 0;
    virtual bool canConnectTo(Connector* other_side) const;
    virtual bool targetsCanConnectTo(Connector* other_side) = 0;
    virtual bool isConnected() = 0;
    virtual bool tryConnect(Connector* other_side) = 0;
    virtual void removeConnection(Connector* other_side) = 0;
    virtual void validateConnections();

    virtual void connectionMovePreview(Connector* other_side) = 0;

    virtual bool isOutput() const {
        return false;
    }
    virtual bool isInput() const {
        return false;
    }

    virtual bool isForwarding() const;

    std::string UUID();

    virtual QPoint centerPoint();

    QString cssClass() {
        return QString("Connector");
    }

    void setLabel(const std::string& label);
    std::string getLabel() const;

    void setType(ConnectionType::ConstPtr type);
    ConnectionType::ConstPtr getType() const;

    void setMinimizedSize(bool mini);
    bool isMinimizedSize() const;

    virtual Command::Ptr removeAllConnectionsCmd() = 0;

public Q_SLOTS:
    virtual bool tryConnect(QObject* other_side);
    virtual void removeConnection(QObject* other_side);

    void removeAllConnectionsUndoable();

    virtual void disable();
    virtual void enable();


Q_SIGNALS:
    void disconnected(QObject*);
    void disabled(Connector* source);
    void enabled(Connector* source);
    void connectionStart();
    void connectionInProgress(Connector*, Connector*);
    void connectionDone();

Q_SIGNALS:
    void connectionFormed(Connector*, Connector*);
    void connectionDestroyed(Connector*, Connector*);
    void messageSent(Connector* source);
    void messageArrived(Connector* source);

protected:
    Connector(Box* parent, const std::string &uuid);
    Connector(Box* parent, int sub_id, int type);
    virtual ~Connector();
    void init(Box* parent);

    virtual void removeAllConnectionsNotUndoable() = 0;

    void errorEvent(bool error, ErrorLevel level);
    void paintEvent(QPaintEvent* event);

protected:
    virtual void findParents();
    virtual QPoint topLeft();

    virtual bool shouldMove(bool left, bool right);
    virtual bool shouldCreate(bool left, bool right);

protected:
    QWidget* parent_widget;
    csapex::DesignBoard* designer;

    Qt::MouseButtons buttons_down_;

    std::string uuid_;
    std::string label_;

    ConnectionType::ConstPtr type_;

    bool minimized_;
};

}

#endif // CONNECTOR_H
