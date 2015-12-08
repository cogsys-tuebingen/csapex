/// HEADER
#include <csapex/view/widgets/movable_graphics_proxy_widget.h>

/// COMPONENT
#include <csapex/view/node/box.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/view/designer/designer_view.h>
#include <csapex/view/designer/widget_controller.h>
#include <csapex/model/node_state.h>

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
    setFlag(ItemSendsGeometryChanges);

    setWidget(box_);

    setAcceptDrops(true);
}

QVariant MovableGraphicsProxyWidget::itemChange(GraphicsItemChange change, const QVariant &value)
{
    if (change == ItemPositionChange) {
        QPointF newPos = value.toPointF();
        if(QApplication::mouseButtons() == Qt::LeftButton){
            if(widget_ctrl_->isGridLockEnabled()) {
                newPos.setX(round(newPos.x() / 10.0) * 10.0);
                newPos.setY(round(newPos.y() / 10.0) * 10.0);
            }
        }
        return newPos;
    }
    else
        return QGraphicsProxyWidget::itemChange(change, value);
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

    if(event->type() == QEvent::GraphicsSceneMousePress) {
        static unsigned long nextz = 1;
        setZValue(nextz++);
    }

    bool do_relay = !ctrl && child && child->objectName() != "boxframe" && strcmp(child->metaObject()->className(), "QLabel");

    before_ = pos();
    if(do_relay) {
        QGraphicsProxyWidget::mousePressEvent(event);
        relay_ = true;

    } else if(shift) {
        clone_start_ = event->pos();
        clone_p_ = true;
    }
}

void MovableGraphicsProxyWidget::signalMoved(const QPointF &delta)
{
    moved(delta.x(), delta.y());
}


void MovableGraphicsProxyWidget::signalMoving(const QPointF &delta)
{
    if(delta != last_signaled_) {
        moving(delta.x(), delta.y());
    }

    last_signaled_ = delta;
}

void MovableGraphicsProxyWidget::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    clone_p_ = false;

    QGraphicsItem::mouseReleaseEvent(event);

    if(relay_) { // child && child->objectName() != "boxframe" && strcmp(child->metaObject()->className(), "QLabel")) {
        QGraphicsProxyWidget::mouseReleaseEvent(event);
        relay_ = false;

    }

    QPointF after = pos();
    if(before_ != after) {
        signalMoved(after - before_);
    }
}

void MovableGraphicsProxyWidget::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if(clone_p_) {
        QPointF delta = clone_start_ - event->pos();
        if(hypot(delta.x(), delta.y()) > 10) {
            NodeStatePtr state = box_->getNodeWorker()->getNodeStateCopy();
            state->setLabel("");
            widget_ctrl_->startPlacingBox(view_, box_->getNodeWorker()->getType(), state);
        }
        return;
    }

    if(relay_) {
        QGraphicsProxyWidget::mouseMoveEvent(event);
    }
    if(!event->isAccepted()) {
        QGraphicsItem::mouseMoveEvent(event);
        signalMoving(pos() - before_);
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
    if (!e || !hasFocus()) {
        return;
    }

    QPointF pos = e->scenePos();
    QPoint globalPos = e->scenePos().toPoint();

    auto target = box_->childAt(e->pos().toPoint());

    QContextMenuEvent contextMenuEvent(QContextMenuEvent::Reason(e->reason()),
                                       pos.toPoint(), globalPos, e->modifiers());
    QApplication::sendEvent(target, &contextMenuEvent);

    e->setAccepted(contextMenuEvent.isAccepted());
}

NodeBox* MovableGraphicsProxyWidget::getBox()
{
    return box_;
}
/// MOC
#include "../../../include/csapex/view/widgets/moc_movable_graphics_proxy_widget.cpp"
