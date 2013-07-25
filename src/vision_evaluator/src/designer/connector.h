#ifndef CONNECTOR_H
#define CONNECTOR_H

/// COMPONENT
#include "connection_type.h"
#include "command.h"
#include "displayable.h"

/// FORWARDS DECLARATION
namespace vision_evaluator
{
class Box;
class DesignBoard;

class Connector : public Displayable
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
    static const QString MIME_CREATE;
    static const QString MIME_MOVE;

public:
    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseMoveEvent(QMouseEvent * e);
    virtual void mouseReleaseEvent(QMouseEvent* e);

    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);

    virtual bool canConnect() = 0;
    virtual bool canConnectTo(Connector* other_side);
    virtual bool isConnected() = 0;
    virtual bool tryConnect(Connector* other_side) = 0;
    virtual void removeConnection(Connector* other_side) = 0;
    virtual void validateConnections();

    virtual bool isOutput() const {
        return false;
    }
    virtual bool isInput() const {
        return false;
    }

    void setUUID(const std::string& uuid);
    std::string UUID();

    virtual QPoint centerPoint();

    int connectorID();

    QString cssClass() {
        return QString("Connector");
    }

    void setLabel(const std::string& label);
    std::string getLabel() const;

    void setType(ConnectionType::Ptr type);
    ConnectionType::ConstPtr getType() const;

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

protected:
    Connector(Box* parent, const std::string& type, int sub_id);
    virtual ~Connector();

    virtual void removeAllConnectionsNotUndoable() = 0;
    virtual Command::Ptr removeAllConnectionsCmd() = 0;

    void errorEvent(bool error);

protected:
    virtual void findParents();
    virtual QPoint topLeft();

protected:
    QWidget* parent_widget;
    vision_evaluator::DesignBoard* designer;

    std::string uuid_;
    std::string label_;

    ConnectionType::Ptr type_;

    Qt::MouseButtons buttons_down_;
};

}

#endif // CONNECTOR_H
