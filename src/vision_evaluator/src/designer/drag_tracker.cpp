/// HEADER
#include "drag_tracker.h"

/// SYSTEM
#include <iostream>

DragTracker::DragTracker(QWidget *parent)
    : callback(parent), overlay_(NULL)
{
    dragging = false;
}

DragTracker::~DragTracker()
{

}
void DragTracker::mousePressEvent(QMouseEvent *e)
{
    if((e->buttons() & Qt::LeftButton)){

        e->accept();

        QPoint current = e->globalPos();

        if(!dragging){
            start_global_mouse_pos = current;
            start_pos = getPos();
            dragging = true;
        }
    }
}

void DragTracker::mouseReleaseEvent(QMouseEvent *e)
{
    if(e->button() & Qt::LeftButton) {
        e->accept();
        dragging = false;
    }
}

void DragTracker::mouseMoveEvent(QMouseEvent *e)
{
    if(dragging){
        e->accept();

        QPoint delta = e->globalPos() - start_global_mouse_pos;

        mouseDelta(delta);
    }
}
