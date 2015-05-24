/// HEADER
#include <csapex/view/profiling_widget.h>

/// COMPONENT
#include <csapex/view/box.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/utility/timer.h>
#include <csapex/core/settings.h>
#include <csapex/view/designer_view.h>
#include <csapex/utility/color.hpp>

/// SYSTEM
#include <QPainter>
#include <QGraphicsProxyWidget>

using namespace csapex;

ProfilingWidget::ProfilingWidget(DesignerView */*view*/, NodeBox *box, QWidget *parent)
    : QWidget(parent), box_(box), node_worker_(box->getNodeWorker()),
      timer_history_pos_(0),
      count_(0)
{
    w_ = 300;
    h_ = 80;

    left_space = 50;
    padding = 5;
    line_height = 14.f;

    //    timer_history_length = settings_.get<int>("timer_history_length", 15);
    timer_history_length = 15;
    timer_history_.resize(timer_history_length);
    apex_assert_hard(timer_history_.size() == timer_history_length);
    apex_assert_hard(timer_history_.capacity() == timer_history_length);

    setFixedSize(w_, h_);

    connect(box_, SIGNAL(destroyed()), this, SLOT(close()));
    connect(box_, SIGNAL(destroyed()), this, SLOT(deleteLater()));

    connection_ = node_worker_->messages_processed.connect(boost::bind(&ProfilingWidget::update, this));
}

ProfilingWidget::~ProfilingWidget()
{
    connection_.disconnect();
}

void ProfilingWidget::reposition(double, double)
{
    QPointF pos = box_->graphicsProxyWidget()->pos() + QPointF(0,box_->height());
    graphicsProxyWidget()->setPos(pos);
}

void ProfilingWidget::paintEvent(QPaintEvent *)
{
    auto new_measurements = node_worker_->extractLatestTimers();
    for(TimerPtr timer : new_measurements) {
        timer_history_[timer_history_pos_] = timer;

        if(++timer_history_pos_ >= (int) timer_history_.size()) {
            timer_history_pos_ = 0;
        }
    }

    QPainter p(this);

    left = padding + left_space;
    right = w_ - padding;
    up = padding;
    bottom = h_ - padding;

    std::size_t history_length = timer_history_.size();

    content_width_ = right - left - 2 * padding;
    indiv_width_ = content_width_ / history_length;
    content_height_ = bottom - up - 2 * padding;

    int n = history_length;

    int max_time_ms = 10;
    const std::vector<Timer::Ptr>& h = timer_history_;
    for(std::vector<Timer::Ptr>::const_iterator timer = h.begin(); timer != h.end(); ++timer) {
        const Timer::Ptr& t = *timer;
        if(!t) {
            continue;
        }

        max_time_ms = std::max(max_time_ms, t->root->lengthMs());

        std::vector<std::pair<std::string, int> > names = t->entries();
        for(std::vector<std::pair<std::string, int> >::const_iterator it = names.begin(); it != names.end(); ++it) {
            const std::string& name = it->first;
            std::map<std::string, QColor>::iterator pos = steps_.find(name);
            if(pos == steps_.end()) {
                steps_[name] = color::fromCount(steps_.size()).light();
                setFixedHeight(h_ + (steps_.size()+2) * line_height + padding);
            }
        }
    }

    // background
    QRect rect = contentsRect().adjusted(0,0,-1,-1);

    p.setBrush(QBrush(QColor(20, 20, 20)));
    p.setPen(QPen(Qt::white));
    p.setOpacity(0.6);

    p.drawRect(rect);

    // x-axis
    p.setOpacity(1.0);
    p.setPen(QPen(Qt::black));
    p.drawLine(left, bottom, right, bottom);

    // y-axis
    p.drawLine(left, bottom, left, up);


    if(timer_history_pos_ < 0) {
        // no entries
        QFont font = p.font();
        font.setPixelSize(line_height * 2);
        p.setFont(font);
        p.setPen(QColor(0, 0, 0));
        p.drawText(rect.adjusted(left, 0, 0, 0), Qt::AlignCenter, "no data");
        return;
    }

    // update stats
    {
        const Timer::Ptr& t = timer_history_[timer_history_pos_];
        if(t) {
            std::vector<std::pair<std::string, int> > names = t->entries();
            for(std::vector<std::pair<std::string, int> >::const_iterator it = names.begin(); it != names.end(); ++it) {
                const std::string& name = it->first;
                steps_acc_[name](it->second);
            }
            ++count_;
        }
    }

    // bars
    if(n > 0) {
        std::stringstream txt;
        txt << max_time_ms << " ms";

        QFont font;
        font.setPixelSize(12);
        p.setFont(font);

        QFontMetrics metrics(font);

        float dy = metrics.height();

        QTextOption opt(Qt::AlignRight);

        p.setPen(QPen(QColor(20, 20, 20)));
        p.drawText(QRect(0, up, left -padding, dy), txt.str().c_str(), opt);
        p.drawText(QRect(0, bottom - dy, left - padding, dy), "0 ms", opt);

        max_time_ms_ = std::max(1, max_time_ms);

        current_draw_x = left + padding + (history_length - n) * indiv_width_;
        for(int time = 0; time < n; ++time) {

            static const float min_opacity = 0.25f;

            float op = ((time - timer_history_pos_ + n - 1) % n) / (float) n;
            p.setOpacity(min_opacity + op * (1.0f - min_opacity));

            const Timer::Ptr& timer = timer_history_[time];

            if(timer) {
                paintTimer(p, timer.get());
            }
        }
    }

    // time line
    p.setOpacity(0.8);
    float pos = left + padding + (timer_history_pos_+1) * indiv_width_;
    QPen pen(QColor(255, 20, 20));
    pen.setWidth(3);
    p.setPen(pen);
    p.drawLine(pos, bottom, pos, up);

    // legend
    p.setOpacity(1.0);
    float y = h_;
    float text_x = left_space + 2*padding;
    float text_w = (w_ - text_x) / 2.0f - padding;
    float info_w = w_ - text_w - text_x;
    float info_x = text_x + text_w + padding;

    // label
    QFont font = p.font();
    font.setPixelSize(line_height * 0.8);
    p.setFont(font);
    p.setPen(QColor(0, 0, 0));
    p.drawText(QRectF(text_x, y, text_w, line_height), "name");
    p.drawText(QRectF(info_x , y, info_w / 2.0f, line_height), "mean");
    p.drawText(QRectF(info_x + info_w / 2.0f, y, info_w / 2.0f, line_height), "stddev");
    y += line_height;
    std::stringstream ss;
    ss << "(" << count_ << " frames, [ms])";
    p.drawText(QRectF(info_x, y, info_w, line_height), QString::fromStdString(ss.str()));
    y += line_height;

    // stats
    for(std::map<std::string, QColor>::const_iterator it = steps_.begin(); it != steps_.end(); ++it) {
        const std::string& name = it->first;
        QBrush brush(it->second);
        QPen pen(brush, 2);
        p.setPen(pen);
        p.fillRect(QRectF(text_x - 2*padding - line_height, y, line_height, line_height), it->second);
        p.drawText(QRectF(text_x, y, text_w, line_height), QString::fromStdString(name));

        accumulator::sample_type mean = boost::accumulators::mean(steps_acc_[name]);
        accumulator::sample_type stddev = std::sqrt(boost::accumulators::variance(steps_acc_[name]));

        p.drawText(QRectF(info_x, y, info_w / 2.0f, line_height), QString::number(mean));
        p.drawText(QRectF(info_x + info_w / 2.0f, y, info_w / 2.0f, line_height), QString::number(stddev));
        y += line_height;
    }
}

void ProfilingWidget::paintTimer(QPainter& p, const Timer * timer)
{
    paintInterval(p, timer->root, 0, 0);

    current_draw_x += indiv_width_;
}

float ProfilingWidget::paintInterval(QPainter& p, const Timer::Interval::Ptr& interval, float height_offset, int depth)
{
    int interval_time = interval->lengthMs();

    float f = interval_time / max_time_ms_;
    f = std::max(0.0f, std::min(1.0f, f));
    float height = f * content_height_;

    p.setBrush(QBrush(steps_[interval->name()]));


    float w = indiv_width_ / (depth+1);
    p.drawRect(QRectF(current_draw_x + indiv_width_ - w, bottom - height - height_offset, w, height));

    float h = height_offset;
    for(std::map<std::string, Timer::Interval::Ptr>::const_iterator sub = interval->sub.begin(); sub != interval->sub.end(); ++sub) {
        const Timer::Interval::Ptr& sub_interval = sub->second;
        h += paintInterval(p, sub_interval, h, depth + 1);
    }

    return height;
}
/// MOC
#include "../../include/csapex/view/moc_profiling_widget.cpp"
