#ifndef PROFILING_WIDGET_H
#define PROFILING_WIDGET_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/utility/timer.h>

/// SYSTEM
#include <QWidget>
#include <map>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

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
    float paintInterval(QPainter &p, const csapex::Timer::Interval &interval, float height_offset, int depth);

private:
    DesignerView* view_;
    NodeBox* box_;
    NodeWorker* node_worker_;

    float w_;
    float h_;
    float content_height_ ;

    float left_space;
    float padding;
    float line_height;

    float left;
    float right;
    float up;
    float bottom;

    double max_time_ms_;

    double current_draw_x;

    double content_width_;
    double indiv_width_;

    std::map<std::string, QColor> steps_;

    typedef boost::accumulators::stats<boost::accumulators::tag::variance> stats;
    typedef boost::accumulators::accumulator_set<double, stats > accumulator;
    std::map<std::string, accumulator> steps_acc_;
    unsigned int count_;
};

}

#endif // PROFILING_WIDGET_H
