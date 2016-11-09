/// HEADER
#include <csapex/view/widgets/minimap_widget.h>

/// COMPONENT
#include <csapex/view/designer/designer_scene.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/node/box.h>

/// SYSTEM
#include <QPainter>
#include <QMouseEvent>

using namespace csapex;

MinimapWidget::MinimapWidget()
    : view_(nullptr), scene_(nullptr), dragging_(false)
{
    setMinimumSize(100, 100);
    setMaximumSize(500, 500);

    setVisible(false);
}

void MinimapWidget::display(GraphView *view)
{
    QObject::connect(view, &GraphView::destroyed,  this, &MinimapWidget::disconnectView);

    QObject::connect(view, SIGNAL(viewChanged()), this, SLOT(update()));
    QObject::connect(this, SIGNAL(positionRequest(QPointF)), view, SLOT(centerOnPoint(QPointF)));
    QObject::connect(this, SIGNAL(zoomRequest(QPointF, double)), view, SLOT(zoomAt(QPointF,double)));

    QObject::connect(this, SIGNAL(resizeRequest(QSize)), this, SLOT(doResize()), Qt::QueuedConnection);

    view_ = view;
    scene_ = view_->designerScene();
}

void MinimapWidget::disconnectView()
{
    if(view_) {
        QObject::disconnect(view_);
        QObject::disconnect(this);

        view_ = nullptr;
    }

    setVisible(false);
}

void MinimapWidget::doResize()
{
    setFixedSize(QSize(1+size_screen_.width(), 1+size_screen_.height()));
}

void MinimapWidget::mousePressEvent(QMouseEvent *me)
{
    if(me->buttons() & Qt::LeftButton) {
        emitPositionRequest(me);
    } else if(me->buttons() & Qt::RightButton) {
        dragging_ = true;
        dragging_start_ = pos();
        dragging_offset_ = me->globalPos();
    }
}

void MinimapWidget::mouseReleaseEvent(QMouseEvent */*me*/)
{
    dragging_ = false;
}

void MinimapWidget::mouseMoveEvent(QMouseEvent *me)
{
    if(me->buttons() & Qt::LeftButton) {
        emitPositionRequest(me);
    } else if(me->buttons() & Qt::RightButton) {
        if(dragging_) {
            move(dragging_start_ + me->globalPos() - dragging_offset_);
        }
    }
}

void MinimapWidget::wheelEvent(QWheelEvent *e)
{
    int direction = (e->delta() > 0) ? 1 : -1;

    QPoint pos = scene_to_minimap_.inverted().map(e->pos());
    Q_EMIT zoomRequest(pos, direction * 2.0);
}

void MinimapWidget::emitPositionRequest(QMouseEvent *me)
{
    int x = me->pos().x();
    int y = me->pos().y();
    if(x < 0 || y < 0 || x > width() || y > height()) {
        return;
    }

    QPoint pos = scene_to_minimap_.inverted().map(me->pos());
    Q_EMIT positionRequest(pos);
}

void MinimapWidget::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);

    if(!view_) {
        return;
    }

    auto boxes = view_->boxes();

    int max_length = 300;

    QRectF scene_rect = scene_->sceneRect();
    double scene_width = scene_rect.width();
    double scene_height = scene_rect.height();

    double screen_scale;
    if(scene_width == 0.0 || scene_height == 0.0) {
        screen_scale = 0.0;
        size_screen_ = QSize(0, 0);

    } else {
        double aspect_ratio = scene_width / scene_height;

        if(scene_width > scene_height) {
            size_screen_ = QSize(max_length, max_length / aspect_ratio);
            screen_scale = max_length / scene_width;
        } else {
            size_screen_ = QSize(max_length * aspect_ratio, max_length);
            screen_scale = max_length / scene_height;
        }
    }
    Q_EMIT resizeRequest(size_screen_);


    scene_to_minimap_ = QTransform::fromTranslate(-scene_rect.topLeft().x(), -scene_rect.topLeft().y()) * QTransform::fromScale(screen_scale, screen_scale);

    QPoint tl_map = scene_to_minimap_.map(scene_rect.topLeft()).toPoint();
    QPoint br_map = scene_to_minimap_.map(scene_rect.bottomRight()).toPoint();

    QRectF rect = view_->mapToScene(view_->viewport()->geometry()).boundingRect();
    QPoint tl_view = scene_to_minimap_.map(rect.topLeft()).toPoint();
    QPoint br_view = scene_to_minimap_.map(rect.bottomRight()).toPoint();

    // limit the view to the scene
    if(tl_view.x() < tl_map.x()) {
        tl_view.setX(tl_map.x());
    }
    if(tl_view.y() < tl_map.y()) {
        tl_view.setY(tl_map.y());
    }
    if(br_view.x() > br_map.x()) {
        br_view.setX(br_map.x());
    }
    if(br_view.y() > br_map.y()) {
        br_view.setY(br_map.y());
    }

    // draw background box
    QBrush brush_bg(QColor::fromRgb(255,255,255));
    QPen pen_bg(QBrush(QColor::fromRgb(0,0,0)), 2);
    painter.setPen(pen_bg);
    painter.setBrush(brush_bg);
    painter.drawRect(QRectF(tl_map, br_map));

    // draw boxes
    QBrush brush_box(QColor::fromRgb(100,100,100,255));
    QPen pen_box(QBrush(QColor::fromRgb(0,0,0)), 2);
    painter.setPen(pen_box);
    painter.setBrush(brush_box);
    for (NodeBox *box : boxes) {
        QRectF rect(box->pos(), box->size());
        painter.drawPolygon(scene_to_minimap_.map(rect));
    }

    // draw view rect
    QBrush brush_rect(QColor::fromRgb(0,0,0,0));
    QPen pen_rect(QBrush(QColor::fromRgb(255,0,0)), 1);
    painter.setPen(pen_rect);
    painter.setBrush(brush_rect);
    painter.drawRect(QRectF(tl_view, br_view));
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_minimap_widget.cpp"
