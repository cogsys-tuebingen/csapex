#ifndef CONNECTOR_H
#define CONNECTOR_H

/// COMPONENT
#include "connection_type.h"
#include "command.h"

/// SYSTEM
#include <QRadioButton>

/// FORWARDS DECLARATION
namespace vision_evaluator
{
class Box;
class DesignBoard;

class Connector : public QRadioButton
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
    static const QString MIME_CREATE;
    static const QString MIME_MOVE;

public:
    virtual bool hitButton(const QPoint&) const;
    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseReleaseEvent(QMouseEvent* e);

    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);

    virtual bool canConnect() = 0;
    virtual bool canConnectTo(Connector* other_side);
    virtual bool isConnected() = 0;
    virtual bool tryConnect(Connector* other_side) = 0;
    virtual void removeConnection(Connector* other_side) = 0;

    virtual void paintEvent(QPaintEvent* e);

    virtual bool isOutput() {
        return false;
    }
    virtual bool isInput() {
        return false;
    }

    std::string UUID();
    void setUUID(const std::string& uuid);

    virtual QPoint centerPoint();

    vision_evaluator::Box* box();
    int connectorID();

    QString cssClass() {
        return QString("Connector");
    }

    std::string getLabel() const;
    void setLabel(const std::string& label);

    std::string getTypeName() const;

public Q_SLOTS:
    virtual bool tryConnect(QObject* other_side);
    virtual void removeConnection(QObject* other_side);

    void removeAllConnectionsUndoable();



Q_SIGNALS:
    void disconnected(QObject*);
    void connectionInProgress(Connector*, Connector*);
    void connectionDone();

protected:
    Connector(Box* parent, const std::string& type, int sub_id);
    virtual ~Connector();

    virtual void removeAllConnectionsNotUndoable() = 0;
    virtual Command::Ptr removeAllConnectionsCmd() = 0;

protected:
    virtual void findParents();
    virtual QPoint topLeft();

protected:
    QWidget* parent_widget;
    vision_evaluator::Box* box_;
    vision_evaluator::DesignBoard* designer;

    std::string uuid_;
    std::string label_;
};

}

#endif // CONNECTOR_H
