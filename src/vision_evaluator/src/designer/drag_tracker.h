#ifndef DRAG_TRACKER_H
#define DRAG_TRACKER_H

/// COMPONENT
#include "overlay.h"

/// SYSTEM
#include <QPoint>
#include <QMouseEvent>
#include <QWidget>

class DragTracker
{
public:
    DragTracker(QWidget* parent);
    virtual ~DragTracker();

    virtual void mousePressEvent(QMouseEvent * e);
    virtual void mouseReleaseEvent(QMouseEvent * e);
    virtual void mouseMoveEvent(QMouseEvent* e);

    virtual void mouseDelta(const QPoint& delta) = 0;
    virtual QPoint getPos() const = 0;

    virtual void setOverlay(Overlay* o) {
        overlay_ = o;
    }
protected:
    QWidget* callback;
    Overlay* overlay_;

    QPoint start_global_mouse_pos;
    QPoint start_pos;
    bool dragging;
};


#endif // DRAG_TRACKER_H
