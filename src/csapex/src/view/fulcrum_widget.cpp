/// HEADER
#include <csapex/view/fulcrum_widget.h>

/// COMPONENT
#include <csapex/model/fulcrum.h>

/// SYSTEM
#include <QEvent>
#include <QGraphicsSceneEvent>
#include <QGraphicsScene>
#include <QGraphicsWidget>

using namespace csapex;

FulcrumWidget::FulcrumWidget(Fulcrum *fulcrum, QGraphicsItem *parent)
    : QGraphicsEllipseItem(parent), fulcrum_(fulcrum)
{
    half_size_ = QPointF(10, 10);

    setFlag(QGraphicsItem::ItemIsMovable);
    setFlag(QGraphicsItem::ItemIsSelectable);

    setPos(fulcrum->pos());
    setRect(QRectF(-half_size_, half_size_));

    QObject::connect(fulcrum, SIGNAL(moved(Fulcrum*,bool)), this, SLOT(moved()));
}

void FulcrumWidget::moved()
{
    QPointF pos = fulcrum_->pos();
    if(pos != scenePos()) {
        setPos(pos);
    }
}

bool FulcrumWidget::sceneEvent(QEvent *event)
{
    bool r =  QGraphicsEllipseItem::sceneEvent(event);

    switch(event->type()) {
    case QEvent::GraphicsSceneMouseMove:
        fulcrum_->move(scenePos(), false);
        break;

    case QEvent::GraphicsSceneMouseRelease:
        fulcrum_->move(scenePos(), true);
        break;

    default:
        break;
    }

    return r;
}
