/// HEADER
#include <csapex/view/widgets/activity_timeline_item.h>

/// PROJECT
#include <csapex/profiling/interval.h>
#include <csapex/view/utility/color.hpp>

/// SYSTEM
#include <QPainter>
#include <QGraphicsView>
#include <QStyleOptionGraphicsItem>
#include <QGraphicsSceneEvent>
#include <QGraphicsWidget>
#include <iostream>
#include <QToolTip>

using namespace csapex;

ActivityTimelineItem::ActivityTimelineItem(std::shared_ptr<Interval const> interval)
    : QGraphicsRectItem(nullptr), interval_(interval), selected_interval_(nullptr)
{
    setAcceptHoverEvents(true);

}

void ActivityTimelineItem::refresh()

{

}

void ActivityTimelineItem::hoverEnterEvent(QGraphicsSceneHoverEvent *event)
{
    cursor_ = event->pos();
    update();
}

void ActivityTimelineItem::hoverMoveEvent(QGraphicsSceneHoverEvent *event)
{
    cursor_ = event->pos();
    update();

    QString msg;

    if(selected_interval_) {
        std::string name = selected_interval_->name();
        double duration = (selected_interval_->getEndMicro() - selected_interval_->getStartMicro()) * 1e-3;
        msg = QString("<b>") + QString::fromStdString(name) + "</b>:<br /> " + QString::number(duration) + " ms";

    } else {
        std::string name = interval_->name();
        double duration = (interval_->getEndMicro() - interval_->getStartMicro()) * 1e-3;
        msg = QString("<b>") + QString::fromStdString(name) + "</b>:<br /> " + QString::number(duration) + " ms";
    }

    for(QGraphicsView *view : scene()->views()) {
        if (view->underMouse() || view->hasFocus()) {
            QPointF pos = view->mapToGlobal(view->mapFromScene(event->pos()));
            QToolTip::showText(mapFromItem(this,pos).toPoint(), msg);
        }
    }
}

void ActivityTimelineItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *event)
{
    cursor_ = QPointF();
    selected_interval_ = nullptr;
    update();
}

void ActivityTimelineItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    QGraphicsRectItem::paint(painter, option, widget);

    QRectF valid_rect = option->rect;

    auto pw = pen().width();
    valid_rect.adjust(pw, pw, -pw, -pw);

    count_ = 0;
    paintInterval(*painter, valid_rect, *interval_);
}

void ActivityTimelineItem::paintInterval(QPainter& p, const QRectF& valid_rect,  const Interval& interval)
{
    rect = valid_rect;

    start = interval_->getStartMicro();
    end = interval_->getEndMicro();

    long duration = (end - start);

    res = rect.width() / duration;

    paintInterval(p, interval, 0);
}

void ActivityTimelineItem::paintInterval(QPainter& p, const Interval& interval, int depth)
{
    double h = rect.height() / (depth + 1);

    for(auto sub = interval.sub.begin(); sub != interval.sub.end(); ++sub) {
        const Interval::Ptr& sub_interval = sub->second;

        long sub_start = sub_interval->getStartMicro();
        long sub_end = sub_interval->getEndMicro();

        if(sub_start >= sub_end) {
            continue;
        }

        QRectF sub_rect(rect.x() + (sub_start - start) * res, rect.y() + rect.height() - h, (sub_end - sub_start) * res, h);
        bool is_selected = sub_rect.contains(cursor_);

        QColor color = color::fromCount<QColor>(count_).light();
        ++count_;

        if(is_selected) {
            p.setBrush(QBrush(color.lighter(110), Qt::SolidPattern ));
            p.setPen(QPen (QColor(20, 20, 20), 3, Qt::SolidLine, Qt::RoundCap, Qt::BevelJoin));

            selected_interval_ = sub_interval.get();

        } else {
            p.setBrush(QBrush(color, Qt::Dense4Pattern));
            p.setPen(QPen (QColor(20, 20, 20)));
        }

        p.drawRect(sub_rect);

        paintInterval(p, *sub_interval, depth + 1);
    }
}
