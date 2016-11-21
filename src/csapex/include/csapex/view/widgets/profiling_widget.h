#ifndef PROFILING_WIDGET_H
#define PROFILING_WIDGET_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// PROJECT
#include <csapex/profiling/profiler.h>
#include <csapex/model/observer.h>

/// SYSTEM
#include <QWidget>
#include <map>

class QSpacerItem;
class QVBoxLayout;

namespace csapex
{

class CSAPEX_QT_EXPORT ProfilingWidget : public QWidget, public Observer
{
    Q_OBJECT

public:
    ProfilingWidget(std::shared_ptr<Profiler> profiler, const std::string &profile, QWidget* parent=0);
    ~ProfilingWidget();

public Q_SLOTS:
    void reset();
    void exportCsv();

protected:
    void enterEvent(QEvent* e);
    void mouseMoveEvent(QMouseEvent* me);
    void leaveEvent(QEvent* e);

    void paintEvent(QPaintEvent *);
    void paintInterval(QPainter &p, const Interval &interval);
    float paintInterval(QPainter &p, const Interval &interval, float height_offset, int depth);

private:
    std::shared_ptr<Profiler> profiler_;
    std::string profile_;

    QVBoxLayout* layout_;
    QSpacerItem* space_for_painting_;
    float bar_height_;
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

    QPointF cursor_;

    const Interval* selected_interval_;
};

}

#endif // PROFILING_WIDGET_H
