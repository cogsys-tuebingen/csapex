/// HEADER
#include <csapex/view/widgets/profiling_widget.h>

/// COMPONENT
#include <csapex/profiling/timer.h>
#include <csapex/core/settings.h>
#include <csapex/view/utility/color.hpp>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <QPainter>
#include <QBoxLayout>
#include <QGraphicsProxyWidget>
#include <QPushButton>
#include <QFileDialog>
#include <fstream>
#include <functional>

using namespace csapex;

ProfilingWidget::ProfilingWidget(std::shared_ptr<Profiler> profiler, const std::string& profile_name, QWidget *parent)
    : QWidget(parent),
      profiler_(profiler), profile_(profile_name),
      space_for_painting_(nullptr)
{
    apex_assert_hard(!profile_.empty());

    int min_w = 300;
    bar_height_ = 80;

    left_space = 50;
    padding = 5;
    line_height = 14.f;

    setMinimumHeight(200);

    layout_ = new QVBoxLayout;
    setLayout(layout_);

    space_for_painting_ = new QSpacerItem(min_w, bar_height_);
    layout_->addItem(space_for_painting_);

    auto* buttons_layout = new QHBoxLayout;

    QPushButton* reset = new QPushButton("reset");
    buttons_layout->addWidget(reset);
    connect(reset, &QPushButton::clicked, this, &ProfilingWidget::reset);

    QPushButton* export_csv = new QPushButton("export");
    buttons_layout->addWidget(export_csv);
    connect(export_csv, &QPushButton::clicked, this, &ProfilingWidget::exportCsv);

    layout_->addLayout(buttons_layout);

    const Profile& profile = profiler_->getProfile(profile_);
    std::shared_ptr<Timer> timer = profile.getTimer();
    connections_.emplace_back(timer->finished.connect([this](Interval::Ptr) { update(); }));

    setMouseTracking(true);
}

ProfilingWidget::~ProfilingWidget()
{
}

void ProfilingWidget::reset()
{
    profiler_->reset();
}

void ProfilingWidget::exportCsv()
{
    QString filename = QFileDialog::getSaveFileName(0, "Save CSV File", "", "*.csv", 0, QFileDialog::DontUseNativeDialog);

    if(!filename.isEmpty()) {
        std::ofstream of(filename.toStdString());

        const Profile& profile = profiler_->getProfile(profile_);

        for(std::map<std::string, QColor>::const_iterator it = steps_.begin(); it != steps_.end(); ++it) {
            const std::string& name = it->first;

            ProfilerStats stats = profile.getStats(name);

            of << name << "," << stats.mean << "," << stats.stddev << '\n';
        }
    }
}

void ProfilingWidget::enterEvent(QEvent *e)
{
    cursor_ = QPointF();
}
void ProfilingWidget::leaveEvent(QEvent *e)
{
    cursor_ = QPointF();
    selected_interval_ = nullptr;
    update();
}

void ProfilingWidget::mouseMoveEvent(QMouseEvent *me)
{
    cursor_ = me->pos();
}

void ProfilingWidget::paintEvent(QPaintEvent *)
{
    apex_assert_hard(!profile_.empty());

    // update stats
    QPainter p(this);

    int w = width();
    int h = space_for_painting_->geometry().height();


    left = padding + left_space;
    right = w - padding;
    up = padding;
    bottom = h - 2 * padding - 2 * line_height - line_height * steps_.size();

    bar_height_ = bottom - padding;

    const Profile& profile = profiler_->getProfile(profile_);

    std::size_t history_length = profile.size();

    content_width_ = right - left - 2 * padding;
    indiv_width_ = content_width_ / history_length;
    content_height_ = bottom - up - 2 * padding;

    int n = history_length;

    double max_time_ms = 1;

    for(const auto& interval : profile.getIntervals()) {
        if(!interval) {
            continue;
        }

        max_time_ms = std::max(max_time_ms, interval->lengthMs());

        std::vector<std::pair<std::string, double> > names;
        interval->entries(names);
        for(auto it = names.begin(); it != names.end(); ++it) {
            const std::string& name = it->first;
            std::map<std::string, QColor>::iterator pos = steps_.find(name);
            if(pos == steps_.end()) {
                steps_[name] = color::fromCount<QColor>(steps_.size()).light();
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


    int current_index = profile.getCurrentIndex();
    if(history_length == 0) {
        // no entries
        QFont font = p.font();
        font.setPixelSize(line_height * 2);
        p.setFont(font);
        p.setPen(QColor(0, 0, 0));
        p.drawText(rect.adjusted(left, 0, 0, 0), Qt::AlignCenter, "no data");
        return;
    }

    selected_interval_ = nullptr;

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

        max_time_ms_ = std::max(1.0, max_time_ms);

        current_draw_x = left + padding + (history_length - n) * indiv_width_;
        for(int time = 0; time < n; ++time) {

            static const float min_opacity = 0.25f;

            float op = ((time - current_index + n - 1) % n) / (float) n;
            p.setOpacity(min_opacity + op * (1.0f - min_opacity));

            const Interval::Ptr& interval = profile.getInterval(time);

            if(interval) {
                paintInterval(p, *interval);
            }
        }

        if(selected_interval_) {
            std::string name = selected_interval_->name();
            ProfilerStats stats = profile.getStats(name);
            setToolTip(QString("<b>") + QString::fromStdString(name) + "</b>:<br /> " + QString::number(stats.mean) + " &plusmn; " + QString::number(stats.stddev) + " ms");
        }
    }

    // time line
    p.setOpacity(0.8);
    float pos = left + padding + (current_index+1) * indiv_width_;
    QPen pen(QColor(255, 20, 20));
    pen.setWidth(3);
    p.setPen(pen);
    p.drawLine(pos, bottom, pos, up);

    // legend
    p.setOpacity(1.0);
    float y = bottom;
    float text_x = left_space + 2*padding;
    float text_w = (w - text_x) / 2.0f - padding;
    float info_w = w - text_w - text_x;
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
    ss << "(" << profile.count() << " frames, [ms])";
    p.drawText(QRectF(info_x, y, info_w, line_height), QString::fromStdString(ss.str()));
    y += line_height;

    // stats
    for(std::map<std::string, QColor>::const_iterator it = steps_.begin(); it != steps_.end(); ++it) {
        const std::string& name = it->first;
        QColor color;
        if(selected_interval_) {
            if(name == selected_interval_->name()) {
                color = it->second;
            } else {
                color = Qt::gray;
            }
        } else {
            color = it->second;
        }
        QBrush brush(color);
        QPen pen(brush, 2);
        p.setPen(pen);
        p.fillRect(QRectF(text_x - 2*padding - line_height, y, line_height, line_height), it->second);
        p.drawText(QRectF(text_x, y, text_w, line_height), QString::fromStdString(name));

        ProfilerStats stats = profile.getStats(name);

        p.drawText(QRectF(info_x, y, info_w / 2.0f, line_height), QString::number(stats.mean));
        p.drawText(QRectF(info_x + info_w / 2.0f, y, info_w / 2.0f, line_height), QString::number(stats.stddev));
        y += line_height;
    }

    // resize to fit content
    if(space_for_painting_->geometry().height() != y) {
        space_for_painting_->changeSize(0, y, QSizePolicy::MinimumExpanding,  QSizePolicy::MinimumExpanding);
        layout()->invalidate();
    }
}

void ProfilingWidget::paintInterval(QPainter& p, const Interval& interval)
{
    paintInterval(p, interval, 0, 0);

    current_draw_x += indiv_width_;
}

float ProfilingWidget::paintInterval(QPainter& p, const Interval& interval, float height_offset, int depth)
{
    float interval_time = interval.lengthMs();

    float f = interval_time / max_time_ms_;
    f = std::max(0.0f, std::min(1.0f, f));
    float height = f * content_height_;

    float w = indiv_width_ / (depth+1);

    QRectF rect(current_draw_x + indiv_width_ - w, bottom - height - height_offset, w, height);
    bool is_selected = rect.contains(cursor_);

    if(is_selected) {
        p.setBrush(QBrush(steps_[interval.name()].lighter(110)));
        p.setPen(QPen (QColor(20, 20, 20), 5, Qt::SolidLine, Qt::RoundCap, Qt::BevelJoin));

    } else {
        p.setBrush(QBrush(steps_[interval.name()]));
        p.setPen(QPen (QColor(20, 20, 20)));
    }

    p.drawRect(rect);

    if(is_selected) {
        selected_interval_ = &interval;
    }

    float h = height_offset;
    for(auto sub = interval.sub.begin(); sub != interval.sub.end(); ++sub) {
        const Interval::Ptr& sub_interval = sub->second;
        h += paintInterval(p, *sub_interval, h, depth + 1);
    }

    return height;
}
/// MOC
#include "../../../include/csapex/view/widgets/moc_profiling_widget.cpp"
