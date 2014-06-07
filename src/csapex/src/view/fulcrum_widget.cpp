/// HEADER
#include <csapex/view/fulcrum_widget.h>

/// COMPONENT
#include <csapex/model/fulcrum.h>
#include <csapex/view/fulcrum_handle.h>

/// SYSTEM
#include <QEvent>
#include <QGraphicsSceneEvent>
#include <QGraphicsScene>
#include <QGraphicsWidget>
#include <QGraphicsLineItem>
#include <cmath>

using namespace csapex;

FulcrumWidget::FulcrumWidget(Fulcrum *fulcrum, QGraphicsItem *parent)
    : QGraphicsEllipseItem(parent), fulcrum_(fulcrum)
{
    half_size_ = QPointF(10, 10);

    setFlag(QGraphicsItem::ItemIsMovable);
//    setFlag(QGraphicsItem::ItemIsSelectable);

    setPos(fulcrum->pos());
    setRect(QRectF(-half_size_, half_size_));

    QObject::connect(fulcrum, SIGNAL(moved(Fulcrum*,bool)), this, SLOT(moved()));

    handle_in_ = new FulcrumHandle(fulcrum->handleIn(), this);
    handle_out_ = new FulcrumHandle(fulcrum_->handleOut(), this);

    line_in = new QGraphicsLineItem(QLineF(QPointF(), handle_in_->pos()), this);
    line_out = new QGraphicsLineItem(QLineF(QPointF(), handle_out_->pos()), this);

    QObject::connect(handle_in_, SIGNAL(moved(bool)), this, SLOT(updateHandleIn(bool)));
    QObject::connect(handle_out_, SIGNAL(moved(bool)), this, SLOT(updateHandleOut(bool)));

    QObject::connect(fulcrum, SIGNAL(movedHandle(Fulcrum*,bool,int)), this, SLOT(updateHandles(Fulcrum*,bool,int)));
}

void FulcrumWidget::moved()
{
    QPointF pos = fulcrum_->pos();
    if(pos != scenePos()) {
        setPos(pos);
    }
}

void FulcrumWidget::updateHandlesHelper(FulcrumHandle& a, QGraphicsLineItem* linea, FulcrumHandle& b, QGraphicsLineItem* lineb, bool dropped)
{
    QPointF pos = a.pos();
    double angle = std::atan2(pos.y(), pos.x());

    linea->setLine(QLineF(QPointF(), pos));

    QPointF other_pos = b.pos();

    double other_angle = angle + M_PI;
    double r = hypot(other_pos.x(), other_pos.y());
    other_pos = QPointF(r*std::cos(other_angle), r*std::sin(other_angle));

    b.setPos(other_pos);
    lineb->setLine(QLineF(QPointF(), other_pos));


    fulcrum_->moveHandles(handle_in_->pos(), handle_out_->pos(), dropped);
}

void FulcrumWidget::updateHandles(Fulcrum* f, bool dropped, int /*which*/)
{
    if(f->handleIn() != handle_in_->pos() || f->handleOut() != handle_out_->pos()) {
        handle_in_->setPos(f->handleIn());
        handle_out_->setPos(f->handleOut());
        updateHandleIn(dropped);
    }
}

void FulcrumWidget::updateHandleIn(bool dropped)
{
    updateHandlesHelper(*handle_in_, line_in, *handle_out_, line_out, dropped);
}

void FulcrumWidget::updateHandleOut(bool dropped)
{
    updateHandlesHelper(*handle_out_, line_in, *handle_in_, line_out, dropped);
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
