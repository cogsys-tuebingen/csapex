#ifndef CONNECTOR_H
#define CONNECTOR_H

/// COMPONENT
#include "overlay.h"
#include "connection_type.h"

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
    static const QString MIME;

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

    virtual void setOverlay(Overlay* o) {
        overlay_ = o;
    }

    std::string UUID();
    void setUUID(const std::string& uuid);

    virtual QPoint centerPoint();

    vision_evaluator::Box* box();
    int connectorID();

    QString cssClass() {
        return QString("Connector");
    }

public Q_SLOTS:
    virtual bool tryConnect(QObject* other_side);
    virtual void removeConnection(QObject* other_side);

    virtual void removeAllConnections() = 0;

Q_SIGNALS:
    virtual void disconnected(QObject*);
    virtual void connectionChanged();

protected:
    Connector(Box* parent, const std::string& type, int sub_id);
    virtual ~Connector();

protected:
    virtual void findParents();
    virtual QPoint topLeft();

protected:
    QWidget* parent_widget;
    vision_evaluator::Box* box_;
    vision_evaluator::DesignBoard* designer;

    Overlay* overlay_;

    std::string uuid_;
};

}

#endif // CONNECTOR_H
