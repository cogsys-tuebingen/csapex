#ifndef THREAD_GROUP_PROFILING_WIDGET_H
#define THREAD_GROUP_PROFILING_WIDGET_H

/// PROJECT
#include <csapex/utility/cpu_affinity.h>
#include <csapex/utility/utility_fwd.h>
#include <csapex/model/observer.h>
#include <csapex/scheduling/scheduling_fwd.h>

/// SYSTEM
#include <QWidget>
#include <QMetaType>
#include <QPointF>
#include <QVector>
#include <memory>
#include <map>

namespace csapex
{

class ThreadGroupProfilingRendererGlobalState
{
public:
    static ThreadGroupProfilingRendererGlobalState& instance()
    {
        static ThreadGroupProfilingRendererGlobalState i;
        return i;
    }

    std::map<const ThreadGroup*, std::string> highlight;
};

class ThreadGroupProfilingRenderer
{
public:
    ThreadGroupProfilingRenderer();
    ThreadGroupProfilingRenderer(const ThreadGroup* profiler);

    void paint(QPainter *painter, const QRect &rect,
               const QPalette &palette, const QPoint cursor) const;
    QSize sizeHint() const;
    const ThreadGroup *getThreadGroup();

private:
    const ThreadGroup* thread_group_;
};


class ThreadGroupProfilingWidget : public QWidget, public Observer
{
    Q_OBJECT

public:
    ThreadGroupProfilingWidget(const ThreadGroup* profiler, QWidget *parent = 0);

    QSize sizeHint() const override;
    void setRenderer(const ThreadGroupProfilingRenderer &renderer) {
        renderer_ = renderer;
    }
    ThreadGroupProfilingRenderer getWidget() { return renderer_; }


protected:
    void paintEvent(QPaintEvent *event) override;

private:
    const ThreadGroup* thread_group_;
    ThreadGroupProfilingRenderer renderer_;
};

}

Q_DECLARE_METATYPE(csapex::ThreadGroupProfilingRenderer)

#endif // THREAD_GROUP_PROFILING_WIDGET_H
