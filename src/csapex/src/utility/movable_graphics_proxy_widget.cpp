/// HEADER
#include <csapex/utility/movable_graphics_proxy_widget.h>

/// COMPONENT
#include <csapex/view/box.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/node.h>
#include <csapex/view/designer_view.h>

/// SYSTEM
#include <QGraphicsSceneMouseEvent>
#include <QWidget>
#include <iostream>
#include <QApplication>

using namespace csapex;

MovableGraphicsProxyWidget::MovableGraphicsProxyWidget(NodeBox *box, DesignerView *view, WidgetController* widget_ctrl, QGraphicsItem *parent, Qt::WindowFlags wFlags)
    : QGraphicsProxyWidget(parent, wFlags), box_(box), view_(view), widget_ctrl_(widget_ctrl), relay_(false), clone_p_(false)
{
    setFlag(ItemIsMovable);
    setFlag(ItemIsSelectable);

    setWidget(box_);

    setAcceptDrops(true);
}

void MovableGraphicsProxyWidget::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    clone_p_ = false;

    QPoint pt = event->pos().toPoint();
    QWidget* child = widget()->childAt(pt);

    bool ctrl = Qt::ControlModifier & QApplication::keyboardModifiers();
    bool shift = Qt::ShiftModifier & QApplication::keyboardModifiers();

    if(!shift) {
        QGraphicsItem::mousePressEvent(event);
    }

    bool do_relay = !ctrl && child && child->objectName() != "boxframe" && strcmp(child->metaObject()->className(), "QLabel");

    if(do_relay) {
        QGraphicsProxyWidget::mousePressEvent(event);
        relay_ = true;
        before_ = pos();

    } else if(shift) {
        clone_start_ = event->pos();
        clone_p_ = true;
    }
}

void MovableGraphicsProxyWidget::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    clone_p_ = false;

//    QPoint pt = event->pos().toPoint();
//    QWidget* child = widget()->childAt(pt);

    QGraphicsItem::mouseReleaseEvent(event);

    if(relay_) { // child && child->objectName() != "boxframe" && strcmp(child->metaObject()->className(), "QLabel")) {
        QGraphicsProxyWidget::mouseReleaseEvent(event);
        relay_ = false;        

        QPointF after = pos();
        if(before_ != after) {
            QPointF delta = after - before_;
            moved(delta.x(), delta.y());
        }
    }
}

void MovableGraphicsProxyWidget::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
//    QPoint pt = event->pos().toPoint();
//    QWidget* child = widget()->childAt(pt);

    if(clone_p_) {
        QPointF delta = clone_start_ - event->pos();
        if(hypot(delta.x(), delta.y()) > 10) {
            BoxManager::instance().startPlacingBox(view_, box_->getNode()->getType(), widget_ctrl_, box_->getNode()->getNodeState());
        }
        return;
    }

    if(relay_) { //child && child->objectName() != "boxframe" && strcmp(child->metaObject()->className(), "QLabel")) {
        QGraphicsProxyWidget::mouseMoveEvent(event);
    }
    if(!event->isAccepted()) {
        QGraphicsItem::mouseMoveEvent(event);
        QPointF delta = pos() - before_;
        moving(delta.x(), delta.y());
    }
}

void MovableGraphicsProxyWidget::dragEnterEvent(QGraphicsSceneDragDropEvent* e)
{
    QGraphicsProxyWidget::dragEnterEvent(e);
}

void MovableGraphicsProxyWidget::dragMoveEvent(QGraphicsSceneDragDropEvent* e)
{
    QGraphicsProxyWidget::dragMoveEvent(e);
}

void MovableGraphicsProxyWidget::dropEvent(QGraphicsSceneDragDropEvent* e)
{
    QGraphicsProxyWidget::dropEvent(e);
}

void MovableGraphicsProxyWidget::dragLeaveEvent(QGraphicsSceneDragDropEvent * e)
{
    QGraphicsProxyWidget::dragLeaveEvent(e);
}

void MovableGraphicsProxyWidget::contextMenuEvent(QGraphicsSceneContextMenuEvent* e)
{
    QGraphicsProxyWidget::contextMenuEvent(e);
}

NodeBox* MovableGraphicsProxyWidget::getBox()
{
    return box_;
}
