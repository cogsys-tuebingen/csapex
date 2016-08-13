#if WIN32
#define _USE_MATH_DEFINES
#include <cmath>
#endif

/// HEADER
#include <csapex/view/designer/fulcrum_widget.h>

/// COMPONENT
#include <csapex/model/fulcrum.h>
#include <csapex/view/designer/fulcrum_handle.h>

/// SYSTEM
#include <QEvent>
#include <QGraphicsSceneEvent>
#include <QGraphicsScene>
#include <QGraphicsWidget>
#include <QGraphicsLineItem>
#include <cmath>
#include <QMenu>
#include <QPainter>

using namespace csapex;

namespace {
QPointF convert(const Point& p) {
    return QPointF(p.x, p.y);
}
Point convert(const QPointF& p) {
    return Point(p.x(), p.y());
}
}

FulcrumWidget::FulcrumWidget(Fulcrum *fulcrum, QGraphicsItem *parent)
    : QGraphicsEllipseItem(parent), fulcrum_(fulcrum)
{
    half_size_ = QPointF(10, 10);

    setFlag(QGraphicsItem::ItemIsMovable);

    setPos(convert(fulcrum->pos()));
    setRect(QRectF(-half_size_, half_size_));

    fulcrum->moved.connect(std::bind(&FulcrumWidget::movedEvent, this));
    fulcrum->movedHandle.connect(std::bind(&FulcrumWidget::movedEvent, this));
    QObject::connect(this, SIGNAL(movedEvent()), this, SLOT(moved()));
    QObject::connect(this, SIGNAL(movedHandlesEvent(Fulcrum*,bool,int)), this, SLOT(updateHandles(Fulcrum*,bool,int)));

    handle_in_ = new FulcrumHandle(convert(fulcrum->handleIn()), this);
    handle_out_ = new FulcrumHandle(convert(fulcrum->handleOut()), this);

    line_in = new QGraphicsLineItem(QLineF(QPointF(), handle_in_->pos()), this);
    line_out = new QGraphicsLineItem(QLineF(QPointF(), handle_out_->pos()), this);

    handle_in_->hide();
    handle_out_->hide();
    line_in->hide();
    line_out->hide();

    QObject::connect(handle_in_, SIGNAL(moved(bool)), this, SLOT(updateHandleIn(bool)));
    QObject::connect(handle_out_, SIGNAL(moved(bool)), this, SLOT(updateHandleOut(bool)));

}


void FulcrumWidget::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    painter->setRenderHint(QPainter::Antialiasing);
    QGraphicsEllipseItem::paint(painter, option, widget);
}

void FulcrumWidget::moved()
{
    QPointF pos = convert(fulcrum_->pos());
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


    fulcrum_->moveHandles(convert(handle_in_->pos()), convert(handle_out_->pos()), dropped);
}

void FulcrumWidget::updateHandles(Fulcrum* f, bool dropped, int /*which*/)
{
    if(convert(f->handleIn()) != handle_in_->pos() || convert(f->handleOut()) != handle_out_->pos()) {
        handle_in_->setPos(convert(f->handleIn()));
        handle_out_->setPos(convert(f->handleOut()));
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

void FulcrumWidget::contextMenuEvent(QGraphicsSceneContextMenuEvent *e)
{
    QMenu menu;
    QAction* del = new QAction("delete fulcrum", &menu);
    menu.addAction(del);

    QMenu type("change type");

    QAction* curve = new QAction("curve", &menu);
    curve->setCheckable(true);
    if(fulcrum_->type() == Fulcrum::FULCRUM_CURVE) {
        curve->setDisabled(true);
        curve->setChecked(true);
    }
    type.addAction(curve);

    QAction* linear = new QAction("linear", &menu);
    linear->setCheckable(true);
    if(fulcrum_->type() == Fulcrum::FULCRUM_LINEAR) {
        linear->setDisabled(true);
        linear->setChecked(true);
    }
    type.addAction(linear);

    menu.addMenu(&type);

    e->accept();
    QAction* selectedItem = menu.exec(QCursor::pos());

    if(selectedItem == del) {
        Q_EMIT deleteRequest(fulcrum_);

    } else if(selectedItem == curve || selectedItem == linear) {
        int type = 0;
        if(selectedItem == curve) {
            type = Fulcrum::FULCRUM_CURVE;

        } else if(selectedItem == linear) {
            type = Fulcrum::FULCRUM_LINEAR;

        } else {
            return;
        }

        Q_EMIT modifyRequest(fulcrum_, (int) type);
    }
}

bool FulcrumWidget::sceneEvent(QEvent *event)
{
    bool r =  QGraphicsEllipseItem::sceneEvent(event);

    switch(event->type()) {
    case QEvent::GraphicsSceneMousePress:
        if(fulcrum_->type() == Fulcrum::FULCRUM_CURVE) {
            bool v = !handle_in_->isVisible();
            handle_in_->setVisible(v);
            handle_out_->setVisible(v);
            line_in->setVisible(v);
            line_out->setVisible(v);
        }
        break;

    case QEvent::GraphicsSceneMouseMove:
        fulcrum_->move(convert(scenePos()), false);
        break;

    case QEvent::GraphicsSceneMouseRelease:
        fulcrum_->move(convert(scenePos()), true);
        break;

    default:
        break;
    }

    return r;
}
/// MOC
#include "../../../include/csapex/view/designer/moc_fulcrum_widget.cpp"
