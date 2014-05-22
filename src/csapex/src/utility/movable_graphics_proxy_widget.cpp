/// HEADER
#include <csapex/utility/movable_graphics_proxy_widget.h>

/// COMPONENT
#include <csapex/view/box.h>

/// SYSTEM
#include <QGraphicsSceneMouseEvent>
#include <QWidget>
#include <iostream>

using namespace csapex;

MovableGraphicsProxyWidget::MovableGraphicsProxyWidget(NodeBox *box, QGraphicsItem *parent, Qt::WindowFlags wFlags)
    : QGraphicsProxyWidget(parent, wFlags), box_(box)
{
    setFlag(ItemIsMovable);
    setFlag(ItemIsSelectable);

    setWidget(box_);

    setAcceptDrops(true);
}

void MovableGraphicsProxyWidget::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QPoint pt = event->pos().toPoint();
    QWidget* child = widget()->childAt(pt);

    if(child && child->objectName() != "boxframe" && strcmp(child->metaObject()->className(), "QLabel")) {
        QGraphicsProxyWidget::mousePressEvent(event);
    }

    if(!event->isAccepted()) {
        QGraphicsItem::mousePressEvent(event);
    }
}

void MovableGraphicsProxyWidget::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    QPoint pt = event->pos().toPoint();
    QWidget* child = widget()->childAt(pt);

    if(child && child->objectName() != "boxframe" && strcmp(child->metaObject()->className(), "QLabel")) {
        QGraphicsProxyWidget::mouseReleaseEvent(event);
    }
    if(!event->isAccepted()) {
        QGraphicsItem::mouseReleaseEvent(event);
    }
}

void MovableGraphicsProxyWidget::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    QPoint pt = event->pos().toPoint();
    QWidget* child = widget()->childAt(pt);

    if(child && child->objectName() != "boxframe" && strcmp(child->metaObject()->className(), "QLabel")) {
        QGraphicsProxyWidget::mouseMoveEvent(event);
    }
    if(!event->isAccepted()) {
        QGraphicsItem::mouseMoveEvent(event);
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
