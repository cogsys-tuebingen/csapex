/// HEADER
#include <csapex/view/widgets/thread_group_profiling_widget.h>

/// PROJECT
#include <csapex/profiling/profiler.h>
#include <csapex/scheduling/thread_group.h>
#include <csapex/view/utility/color.hpp>

/// SYSTEM
#include <QtWidgets>
#include <iostream>

using namespace csapex;

ThreadGroupProfilingRenderer::ThreadGroupProfilingRenderer(const ThreadGroup* thread_group) : thread_group_(thread_group)
{
}

ThreadGroupProfilingRenderer::ThreadGroupProfilingRenderer()
{
}

const ThreadGroup* ThreadGroupProfilingRenderer::getThreadGroup()
{
    return thread_group_;
}

QSize ThreadGroupProfilingRenderer::sizeHint() const
{
    return QSize(256, 96);
}

void ThreadGroupProfilingRenderer::paint(QPainter* painter, const QRect& rect, const QPalette& palette, const QPoint cursor) const
{
    painter->save();

    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setPen(Qt::NoPen);

    painter->setBrush(palette.foreground());

    ProfilerPtr profiler = thread_group_->getProfiler();
    TimerPtr timer = profiler->getTimer(thread_group_->getName());

    double full_width = rect.width();
    double full_duration_ms = timer->elapsedMs();

    struct Entry
    {
        std::string name;
        int idx;
        double duration_ms;
        int percent;
    };

    int idx = 0;
    std::vector<Entry> segments;
    for (const auto& pair : timer->entries()) {
        Entry e;
        e.name = pair.first;
        e.idx = idx;
        e.duration_ms = pair.second;
        e.percent = std::round(100.0 * e.duration_ms / full_duration_ms);
        ++idx;
        segments.push_back(e);
    }

    std::sort(segments.begin(), segments.end(), [](const Entry& a, const Entry& b) { return a.duration_ms < b.duration_ms; });

    QTextOption text_top(Qt::AlignTop | Qt::AlignHCenter);
    text_top.setWrapMode(QTextOption::NoWrap);

    QTextOption text_bottom(Qt::AlignBottom | Qt::AlignHCenter);

    double next_segment_start = 0;
    for (const Entry& e : segments) {
        double rel_width = e.duration_ms / full_duration_ms * full_width;

        QRect segment_rect(rect.x() + next_segment_start, rect.y(), rel_width, rect.height());

        bool is_selected = segment_rect.contains(cursor);
        if (is_selected) {
            painter->setPen(QPen(QColor(20, 20, 20), 5, Qt::SolidLine, Qt::RoundCap, Qt::BevelJoin));

            std::string highlight = e.name;
            ThreadGroupProfilingRendererGlobalState::instance().highlight[thread_group_] = highlight;

        } else {
            painter->setPen(QPen(QColor(20, 20, 20)));
        }

        QColor col = color::fromCount<QColor>(e.idx);
        painter->setBrush(QBrush(col.light()));
        painter->drawRect(segment_rect);

        painter->setPen(QPen(Qt::black));
        painter->setBrush(QBrush(col));
        painter->drawText(segment_rect, QString::fromStdString(e.name), text_top);

        painter->setPen(QPen(Qt::black));
        painter->setBrush(QBrush(col));
        painter->drawText(segment_rect, QString::number(e.percent) + "%", text_bottom);

        next_segment_start += rel_width;
    }

    QRect segment_rect(rect.x() + next_segment_start, rect.y(), full_width - next_segment_start, rect.height());
    QColor col(Qt::gray);
    painter->setPen(Qt::NoPen);
    painter->setBrush(QBrush(col, Qt::BDiagPattern));
    painter->drawRect(segment_rect);

    painter->restore();
}

ThreadGroupProfilingWidget::ThreadGroupProfilingWidget(const ThreadGroup* thread_group, QWidget* parent) : QWidget(parent), thread_group_(thread_group), renderer_(thread_group_)
{
    setMouseTracking(true);
    setAutoFillBackground(true);

    observe(thread_group_->getProfiler()->updated, [this]() { update(); });
}

QSize ThreadGroupProfilingWidget::sizeHint() const
{
    return renderer_.sizeHint();
}

void ThreadGroupProfilingWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    QPoint pos = cursor().pos();
    renderer_.paint(&painter, rect(), this->palette(), pos);
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_thread_group_profiling_widget.cpp"
