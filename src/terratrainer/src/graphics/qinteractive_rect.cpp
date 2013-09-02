#include "qinteractive_rect.h"
#include "qinteractive_scene.h"
#include <QApplication>
#include <QDrag>
#include <QMimeData>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <iostream>
#include <QGraphicsItem>

QInteractiveItem::QInteractiveItem(const QRectF &rect) :
    QGraphicsRectItem(rect),
    classID_(-1),
    interaction_(INIT)
{
    setFlags(ItemIsSelectable | ItemIsMovable | ItemSendsGeometryChanges);
    setAcceptedMouseButtons(Qt::LeftButton);
}

void QInteractiveItem::setPen(const QPen &pen)
{
    pen_ = pen;
}

void QInteractiveItem::setSelectPen(const QPen &pen)
{
    pen_selected_ = pen;
}

void QInteractiveItem::setClass(const int classID)
{
    classID_ = classID;
}

int QInteractiveItem::getClass()
{
    return classID_;
}

void QInteractiveItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QInteractiveScene *s = dynamic_cast<QInteractiveScene*>(scene());
    event->accept();
    if(sceneBoundingRect().contains(event->scenePos())) {
        switch(s->getMode()) {
        case QInteractiveScene::ADD:
            interaction_ = NONE;
            break;
        case QInteractiveScene::SELECT:
            QGraphicsItem::setSelected(!isSelected());
            break;
        case QInteractiveScene::MOVE:
            interaction_ = MOVING;
            drag_start_mouse_pos_ = event->scenePos();
            drag_start_rect_pos_ = scenePos();
            break;
        default:
            break;
        }
    }
}

void QInteractiveItem::setLimitedPos(const QPointF& pos)
{
    QPointF next_pos(pos);

    QGraphicsScene* s = scene();
    int sw = s->width();
    int sh = s->height();

    int w = boundingRect().width();
    int h = boundingRect().height();

    if(next_pos.x() < 0)
        next_pos.setX(0);
    else if(next_pos.x() + w >= sw)
        next_pos.setX(sw - w);

    if(next_pos.y() < 0)
        next_pos.setY(0);
    else if(next_pos.y() + h >= sh)
        next_pos.setY(sh - h);

    setPos(next_pos);
}

void QInteractiveItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    event->accept();

    if(interaction_ != MOVING) {
        setCursor(Qt::ArrowCursor);
        return;
    }

    QPointF offset = event->scenePos() - drag_start_mouse_pos_;
    setCursor(Qt::ClosedHandCursor);

    if (QLineF(event->screenPos(), event->buttonDownScreenPos(Qt::LeftButton))
            .length() < QApplication::startDragDistance()) {
        return;
    }


    QPointF next_pos = drag_start_rect_pos_ + offset;

    setLimitedPos(next_pos);
}

void QInteractiveItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    event->accept();
    setCursor(Qt::ArrowCursor);
    interaction_ = NONE;
}

void QInteractiveItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    if(isSelected()) {
        painter->setPen(pen_selected_);
    } else {
        painter->setPen(pen_);
    }
    painter->drawRect(QRectF(rect().topLeft(), rect().bottomRight() - QPointF(1,1)));
}
