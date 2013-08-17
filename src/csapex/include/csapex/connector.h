#ifndef CONNECTOR_H
#define CONNECTOR_H

/// COMPONENT
#include <csapex/connection_type.h>
#include <csapex/command.h>
#include <csapex/displayable.h>

/// FORWARDS DECLARATION
namespace csapex
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

    static std::string makeUUID(const std::string &box_uuid, bool in, int sub_id);

public:
    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseMoveEvent(QMouseEvent * e);
    virtual void mouseReleaseEvent(QMouseEvent* e);

    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);

    virtual bool canConnect() = 0;
    virtual bool canConnectTo(Connector* other_side);
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

    void setType(ConnectionType::Ptr type);
    ConnectionType::ConstPtr getType() const;

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

protected:
    Connector(Box* parent, const std::string &uuid);
    virtual ~Connector();

    virtual void removeAllConnectionsNotUndoable() = 0;

    void errorEvent(bool error, ErrorLevel level);
    void paintEvent(QPaintEvent* event);

protected:
    virtual void findParents();
    virtual QPoint topLeft();

protected:
    QWidget* parent_widget;
    csapex::DesignBoard* designer;

    Qt::MouseButtons buttons_down_;

    std::string uuid_;
    std::string label_;

    ConnectionType::Ptr type_;
};

}

#endif // CONNECTOR_H
