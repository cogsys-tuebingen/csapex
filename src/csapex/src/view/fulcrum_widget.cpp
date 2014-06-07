/// HEADER
#include <csapex/view/fulcrum_widget.h>

/// COMPONENT
#include <csapex/model/fulcrum.h>
#include <csapex/view/fulcrum_handle.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/delete_fulcrum.h>
#include <csapex/command/modify_fulcrum.h>

/// SYSTEM
#include <QEvent>
#include <QGraphicsSceneEvent>
#include <QGraphicsScene>
#include <QGraphicsWidget>
#include <QGraphicsLineItem>
#include <cmath>
#include <QMenu>

using namespace csapex;

FulcrumWidget::FulcrumWidget(Fulcrum *fulcrum, CommandDispatcher* dispatcher, QGraphicsItem *parent)
    : QGraphicsEllipseItem(parent), fulcrum_(fulcrum), cmd_dispatcher_(dispatcher)
{
    half_size_ = QPointF(10, 10);

    setFlag(QGraphicsItem::ItemIsMovable);

    setPos(fulcrum->pos());
    setRect(QRectF(-half_size_, half_size_));

    QObject::connect(fulcrum, SIGNAL(moved(Fulcrum*,bool)), this, SLOT(moved()));

    handle_in_ = new FulcrumHandle(fulcrum->handleIn(), this);
    handle_out_ = new FulcrumHandle(fulcrum_->handleOut(), this);

    line_in = new QGraphicsLineItem(QLineF(QPointF(), handle_in_->pos()), this);
    line_out = new QGraphicsLineItem(QLineF(QPointF(), handle_out_->pos()), this);

    handle_in_->hide();
    handle_out_->hide();
    line_in->hide();
    line_out->hide();

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

void FulcrumWidget::contextMenuEvent(QGraphicsSceneContextMenuEvent *e)
{
    QMenu menu;
    QAction* del = new QAction("delete fulcrum", &menu);
    menu.addAction(del);

    QMenu type("change type");

    QAction* curve = new QAction("curve", &menu);
    curve->setCheckable(true);
    if(fulcrum_->type() == Fulcrum::CURVE) {
        curve->setDisabled(true);
        curve->setChecked(true);
    }
    type.addAction(curve);

    QAction* linear = new QAction("linear", &menu);
    linear->setCheckable(true);
    if(fulcrum_->type() == Fulcrum::LINEAR) {
        linear->setDisabled(true);
        linear->setChecked(true);
    }
    type.addAction(linear);

    menu.addMenu(&type);

    e->accept();
    QAction* selectedItem = menu.exec(QCursor::pos());

    if(selectedItem == del) {
        cmd_dispatcher_->execute(Command::Ptr(new command::DeleteFulcrum(fulcrum_->connectionId(), fulcrum_->id())));

    } else if(selectedItem == curve || selectedItem == linear) {
        int type = 0;
        if(selectedItem == curve) {
            type = Fulcrum::CURVE;

        } else if(selectedItem == linear) {
            type = Fulcrum::LINEAR;

        } else {
            return;
        }

        Fulcrum* f = fulcrum_;
        command::ModifyFulcrum::Ptr cmd(new command::ModifyFulcrum(
                                            f->connectionId(), f->id(),
                                            f->type(), f->handleIn(), f->handleOut(),
                                            type, f->handleIn(), f->handleOut()));
        cmd_dispatcher_->execute(cmd);
    }
}

bool FulcrumWidget::sceneEvent(QEvent *event)
{
    bool r =  QGraphicsEllipseItem::sceneEvent(event);

    switch(event->type()) {
    case QEvent::GraphicsSceneMousePress:
        if(fulcrum_->type() == Fulcrum::CURVE) {
            bool v = !handle_in_->isVisible();
            handle_in_->setVisible(v);
            handle_out_->setVisible(v);
            line_in->setVisible(v);
            line_out->setVisible(v);
        }
        break;

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
