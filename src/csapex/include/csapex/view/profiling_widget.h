#ifndef PROFILING_WIDGET_H
#define PROFILING_WIDGET_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/utility/timer.h>

/// SYSTEM
#include <QWidget>

namespace csapex
{

class ProfilingWidget : public QWidget
{
    Q_OBJECT

public:
    ProfilingWidget(DesignerView *view, NodeBox* box, QWidget* parent=0);

public Q_SLOTS:
    void reposition(double x, double y);

protected:
    void paintEvent(QPaintEvent *);
    void paintTimer(QPainter &p, const Timer*);
    void paintInterval(QPainter &p, const csapex::Timer::Interval &interval, int height_offset, int depth);

private:
    DesignerView* view_;
    NodeBox* box_;
    NodeWorker* node_worker_;

    int w_;
    int h_;
    int content_height_ ;

    int left;
    int right;
    int up;
    int bottom;

    double max_time_ms_;

    double current_draw_x;

    double content_width_;
    double indiv_width_;
};

}

#endif // PROFILING_WIDGET_H
