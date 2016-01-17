#ifndef PROFILING_WIDGET_H
#define PROFILING_WIDGET_H

/// COMPONENT
#include <csapex/utility/timer.h>
#include <csapex/view/view_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/utility/utility_fwd.h>
#include <csapex/utility/slim_signal.hpp>

/// SYSTEM
#include <QWidget>
#include <map>
#define BOOST_PARAMETER_MAX_ARITY 7
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

class QSpacerItem;
class QVBoxLayout;

namespace csapex
{

class ProfilingWidget : public QWidget
{
    Q_OBJECT

public:
    ProfilingWidget(GraphView *view, NodeBox* box, QWidget* parent=0);
    ~ProfilingWidget();

public Q_SLOTS:
    void reposition(double x, double y);

    void reset();
    void exportCsv();

protected:
    void paintEvent(QPaintEvent *);
    void paintTimer(QPainter &p, const Timer*);
    float paintInterval(QPainter &p, const Timer::Interval::Ptr &interval, float height_offset, int depth);

private:
    NodeBox* box_;
    NodeWorker* node_worker_;

    csapex::slim_signal::Connection connection_;

    QVBoxLayout* layout_;
    QSpacerItem* space_for_painting_;
    float bar_height_;
    float content_height_ ;

    float left_space;
    float padding;
    float line_height;

    std::size_t timer_history_length;
    int timer_history_pos_;
    std::vector<TimerPtr> timer_history_;

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
