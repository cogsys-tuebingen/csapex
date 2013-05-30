#ifndef CONNECTOR_H
#define CONNECTOR_H

/// COMPONENT
#include "drag_tracker.h"
#include "overlay.h"

/// SYSTEM
#include <QRadioButton>

/// FORWARDS DECLARATION
namespace vision_evaluator {
class Box;
class DesignBoard;
}

class Connector : public QRadioButton, public DragTracker
{
    Q_OBJECT

    virtual void mouseMoveEvent(QMouseEvent* e) {DragTracker::mouseMoveEvent(e);}

public:
    virtual bool hitButton(const QPoint &) const;
    virtual void mousePressEvent(QMouseEvent * e);
    virtual void mouseReleaseEvent(QMouseEvent * e);

    virtual bool canConnect() = 0;
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

    virtual QPoint centerPoint();

public Q_SLOTS:
    virtual bool tryConnect(QObject* other_side);
    virtual void removeConnection(QObject *other_side);

protected:
    Connector(QWidget* parent);
    virtual ~Connector();

protected:
    virtual void findParents();
    virtual void mouseDelta(const QPoint& delta);
    virtual QPoint getPos() const;
    virtual QPoint topLeft();

protected:
    QWidget* parent_widget;
    vision_evaluator::Box* box;
    vision_evaluator::DesignBoard* designer;
};

#endif // CONNECTOR_H
