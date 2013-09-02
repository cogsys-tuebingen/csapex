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
            break;
        default:
            break;
        }
    }
}

void QInteractiveItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    event->accept();

    if(interaction_ != MOVING) {
        setCursor(Qt::ArrowCursor);
        return;
    }

    QPointF offset = event->pos() - event->lastPos();
    setCursor(Qt::ClosedHandCursor);

    if (QLineF(event->screenPos(), event->buttonDownScreenPos(Qt::LeftButton))
            .length() < QApplication::startDragDistance()) {
        return;
    }


    QRectF rect = sceneBoundingRect();
    QPointF p = mapToScene(pos().x(), pos().y());
    QPointF r = mapToItem(this, p);
    std::cout << pos().x() << " +++ " << pos().y() << std::endl;
    std::cout << p.x() << " --- " << p.y() << std::endl;
    std::cout << r.x() << " +-+ " << r.y() << std::endl;

    moveBy(offset.x(), offset.y());
    std::cout << sceneBoundingRect().x() << " " << sceneBoundingRect().y() << std::endl;

    QInteractiveScene *s = dynamic_cast<QInteractiveScene*>(scene());
    if(s != NULL && !s->onBackground(this)) {
        QRectF bounding = sceneBoundingRect();
        qreal x = std::min(s->sceneRect().x(), std::max(0.0, bounding.x() - bounding.width()));
        qreal y = std::min(s->sceneRect().y(), std::max(0.0, bounding.y() - bounding.height()));

        std::cout << " +++ " << mapFromScene(x,y).x() << " " << mapFromScene(x,y).y() << std::endl;
        setPos(x,y);
//        moveBy(-offset.x(), -offset.y());
        interaction_ = STOPPED;
    }
    if(s != NULL) {
        std::cout << s->sceneRect().x() << " " << s->sceneRect().y() << std::endl;
    }

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
    painter->drawRect(rect());
}
