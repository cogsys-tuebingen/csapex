/// HEADER
#include <csapex/view/profiling_widget.h>

/// COMPONENT
#include <csapex/view/box.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/utility/timer.h>
#include <csapex/core/settings.h>

/// SYSTEM
#include <QPainter>

using namespace csapex;

ProfilingWidget::ProfilingWidget(QWidget *parent, NodeBox *box)
    : QWidget(parent), box_(box), node_worker_(box->getNode()->getNodeWorker())
{
    w_ = 300;
    h_ = 80;

    setFixedSize(w_, h_);

    reposition(box_);

    connect(box_, SIGNAL(destroyed()), this, SLOT(close()));
    connect(box_, SIGNAL(destroyed()), this, SLOT(deleteLater()));
    connect(box_, SIGNAL(moved(NodeBox*,int,int)), this, SLOT(reposition(NodeBox*)));
}

void ProfilingWidget::reposition(NodeBox *box)
{
    move(box->pos() + QPoint(0, box->height()));
}

void ProfilingWidget::paintEvent(QPaintEvent *)
{
    QPainter p(this);

    QRect rect = contentsRect().adjusted(0,0,-1,-1);

    p.setBrush(QBrush(QColor(200, 200, 255)));
    p.setPen(QPen(Qt::black));
    p.setOpacity(0.6);

    p.drawRect(rect);

    int padding = 5;

    left = padding + 50;
    right = w_ - padding;
    up = padding;
    bottom = h_ - padding;

    content_width_ = right - left - 2 * padding;
    indiv_width_ = content_width_ / Settings::timer_history_length_;
    content_height_ = bottom - up - 2 * padding;

    p.setPen(QPen(Qt::black));

    // x-axis
    p.drawLine(left, bottom, right, bottom);

    // y-axis
    p.drawLine(left, bottom, left, up);

    size_t max = Settings::timer_history_length_;
    int n = std::min(max, node_worker_->timer_history_.size());

    if(n > 0) {
        int max_time_ms = 10;

        const std::deque<Timer::Ptr>& h = node_worker_->timer_history_;
        for(std::deque<Timer::Ptr>::const_iterator timer = h.begin(); timer != h.end(); ++timer) {
            const Timer::Ptr& t = *timer;
            max_time_ms = std::max(max_time_ms, t->intervals.lengthMs());
        }

        std::stringstream txt;
        txt << max_time_ms << " ms";


        QFont font;
        font.setPixelSize(12);
        p.setFont(font);

        QFontMetrics metrics(font);

        int dy = metrics.height();

        QTextOption opt(Qt::AlignRight);

        p.setPen(QPen(QColor(20, 20, 20)));
        p.drawText(QRect(0, up, left -padding, dy), txt.str().c_str(), opt);
        p.drawText(QRect(0, bottom - dy, left - padding, dy), "0 ms", opt);

        max_time_ms_ = std::max(1, max_time_ms);

        current_draw_x = left + padding + (max - n) * indiv_width_;
        for(int time = 0; time < n; ++time) {
            const Timer::Ptr& timer = node_worker_->timer_history_[time];

            paintTimer(p, timer.get());
        }
    }
}

void ProfilingWidget::paintTimer(QPainter& p, const Timer * timer)
{
    paintInterval(p, timer->intervals, 0, 0);

    current_draw_x += indiv_width_;
}

void ProfilingWidget::paintInterval(QPainter& p, const Timer::Interval& interval, int height_offset, int depth)
{
    int interval_time = interval.lengthMs();

    double f = interval_time / max_time_ms_;
    f = std::max(0.0, std::min(1.0, f));
    int height = f * content_height_;

    int r,g,b;

    if(interval.name == "io") {
        r = 0;
        g = 0;
        b = 0;
    } else {
        r = 255 * ((1 + depth) / 3.0);
        g = 128;
        b = 64;
    }

    p.setBrush(QBrush(QColor(r, g, b)));

    int w = indiv_width_ / (depth+1);

    p.drawRect(current_draw_x + indiv_width_ - w, bottom - height, w, height);

    for(std::vector<Timer::Interval>::const_iterator sub = interval.sub.begin(); sub != interval.sub.end(); ++sub) {
        const Timer::Interval& sub_interval = *sub;
        paintInterval(p, sub_interval, height_offset + height, depth + 1);
    }
}
