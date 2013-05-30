#ifndef CONNECTOR_H
#define CONNECTOR_H

/// COMPONENT
#include "overlay.h"

/// SYSTEM
#include <QRadioButton>

/// FORWARDS DECLARATION
namespace vision_evaluator {
class Box;
class DesignBoard;
}

class Connector : public QRadioButton
{
    Q_OBJECT

public:
    static const QString MIME;

public:
    virtual bool hitButton(const QPoint &) const;
    virtual void mousePressEvent(QMouseEvent * e);

    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent * e);

    virtual bool canConnect() = 0;
    virtual bool canConnectTo(Connector* other_side);
    virtual bool isConnected() = 0;
    virtual bool tryConnect(Connector* other_side) = 0;
    virtual void removeConnection(Connector *other_side) = 0;

    virtual void paintEvent(QPaintEvent *e);

    virtual bool isOutput() {
        return false;
    }
    virtual bool isInput() {
        return false;
    }

    virtual void setOverlay(Overlay* o) {
        overlay_ = o;
    }

    virtual QPoint centerPoint();

public Q_SLOTS:
    virtual bool tryConnect(QObject* other_side);
    virtual void removeConnection(QObject *other_side);

protected:
    Connector(QWidget* parent);
    virtual ~Connector();

protected:
    virtual void findParents();
    virtual QPoint topLeft();

protected:
    QWidget* parent_widget;
    vision_evaluator::Box* box;
    vision_evaluator::DesignBoard* designer;

    Overlay* overlay_;
};

#endif // CONNECTOR_H
