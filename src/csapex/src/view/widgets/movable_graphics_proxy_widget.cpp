/// HEADER
#include <csapex/view/widgets/movable_graphics_proxy_widget.h>

/// COMPONENT
#include <csapex/view/node/box.h>
#include <csapex/view/node/note_box.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/model/node_state.h>
#include <csapex/core/graphio.h>
#include <csapex/core/csapex_core.h>
#include <csapex/model/graph_facade.h>

/// SYSTEM
#include <QGraphicsSceneMouseEvent>
#include <QWidget>
#include <iostream>
#include <QApplication>

using namespace csapex;


long MovableGraphicsProxyWidget::next_box_z = 1;
long MovableGraphicsProxyWidget::next_note_z = std::numeric_limits<int>::min();

MovableGraphicsProxyWidget::MovableGraphicsProxyWidget(NodeBox *box, GraphView *view, CsApexViewCore &view_core, QGraphicsItem *parent, Qt::WindowFlags wFlags)
    : QGraphicsProxyWidget(parent, wFlags), box_(box), view_(view), view_core_(view_core), relay_(false), clone_started_(false), clone_allowed_(false)
{
    setFlag(ItemIsMovable);
    setFlag(ItemIsSelectable);
    setFlag(ItemSendsGeometryChanges);

    setWidget(box_);

    long z = 0;

    NodeHandle* nh = box_->getNodeHandle();
    if(nh) {
        NodeStatePtr state = nh->getNodeState();
        z = state->getZ();
    }

    if(z == 0){
        if(dynamic_cast<NoteBox*>(box_)) {
            z = std::numeric_limits<long>::min();
        } else {
            z = 1;
        }
    }

    if(dynamic_cast<NoteBox*>(box_)) {
        next_note_z = std::max(z, next_note_z);
    } else {
        next_box_z = std::max(z, next_box_z);
    }

    setZValue(z);
}

MovableGraphicsProxyWidget::~MovableGraphicsProxyWidget()
{

}

QVariant MovableGraphicsProxyWidget::itemChange(GraphicsItemChange change, const QVariant &value)
{
    if (QApplication::mouseButtons() == Qt::LeftButton &&
            change == ItemPositionChange &&
            view_core_.isGridLockEnabled()) {

        QVariant value_p = QGraphicsProxyWidget::itemChange(change, value);
        QPointF newPos = value_p.toPointF();
        newPos.setX(round(newPos.x() / 10.0) * 10.0);
        newPos.setY(round(newPos.y() / 10.0) * 10.0);
        return newPos;
    }
    else
        return QGraphicsProxyWidget::itemChange(change, value);
}

void MovableGraphicsProxyWidget::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    clone_allowed_ = false;

    QPoint pt = event->pos().toPoint();
    QWidget* child = widget()->childAt(pt);

//    bool ctrl = Qt::ControlModifier & QApplication::keyboardModifiers();
    bool shift = Qt::ShiftModifier & QApplication::keyboardModifiers();

    if(!shift) {
        QGraphicsItem::mousePressEvent(event);
    }

    if(event->type() == QEvent::GraphicsSceneMousePress) {
        long z;

        if(dynamic_cast<NoteBox*>(box_)) {
            z = ++next_note_z;
        } else {
            z = ++next_box_z;
        }

        setZValue(z);
        NodeHandle* nh = box_->getNodeHandle();
        if(nh) {
            NodeStatePtr state = nh->getNodeState();
            state->setZ(z);
        }
    }

    bool do_relay = child && child->objectName() != "boxframe" &&
            (child->parent()->objectName() != "boxframe" || strcmp(child->metaObject()->className(), "QFrame")) &&
            (child->parent()->objectName() != "header_frame" || strcmp(child->metaObject()->className(), "QLabel"));

    before_ = pos();
    if(do_relay) {
        QGraphicsProxyWidget::mousePressEvent(event);
        relay_ = true;

    } else if(shift) {
        if(!clone_allowed_) {
            clone_start_ = event->pos();
        }
        clone_allowed_ = true;
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
    clone_allowed_ = false;
    clone_started_ = false;

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
    if(clone_allowed_ && !clone_started_) {
        QPointF delta = clone_start_ - event->pos();
        if(hypot(delta.x(), delta.y()) > 10) {
            clone_started_ = true;
            view_->startCloningSelection(box_, -event->pos().toPoint());
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

    if(target) {
        QContextMenuEvent contextMenuEvent(QContextMenuEvent::Reason(e->reason()),
                                           pos.toPoint(), globalPos, e->modifiers());
        QApplication::sendEvent(target, &contextMenuEvent);

        e->setAccepted(contextMenuEvent.isAccepted());
    } else {
        QContextMenuEvent contextMenuEvent(QContextMenuEvent::Reason(e->reason()),
                                           pos.toPoint(), globalPos, e->modifiers());
        QApplication::sendEvent(box_, &contextMenuEvent);

        e->setAccepted(contextMenuEvent.isAccepted());

    }
}

NodeBox* MovableGraphicsProxyWidget::getBox()
{
    return box_;
}
/// MOC
#include "../../../include/csapex/view/widgets/moc_movable_graphics_proxy_widget.cpp"
