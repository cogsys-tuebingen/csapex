#ifndef TRACING_TIMELINE_ITEM_H
#define TRACING_TIMELINE_ITEM_H

/// SYSTEM
#include <QGraphicsRectItem>
#include <QMouseEvent>
#include <memory>

namespace csapex
{
class Interval;

class TracingTimelineItem : public QGraphicsRectItem
{
public:
    explicit TracingTimelineItem(std::shared_ptr<Interval const> interval);

    void refresh();

    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = 0) override;

    virtual void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
    virtual void hoverMoveEvent(QGraphicsSceneHoverEvent* event) override;
    virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;

protected:
    void paintInterval(QPainter& p, const QRectF& valid_rect, const Interval& interval);
    void paintInterval(QPainter& p, const Interval& interval, int depth);

private:
    std::shared_ptr<Interval const> interval_;

    QPointF cursor_;

    int count_;

    QRectF rect;

    long start;
    long end;

    double res;

    const Interval* selected_interval_;
};

}  // namespace csapex

#endif  // TRACING_TIMELINE_ITEM_H
