#ifndef PROFILING_WIDGET_H
#define PROFILING_WIDGET_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QWidget>

namespace csapex
{

class ProfilingWidget : public QWidget
{
    Q_OBJECT

public:
    ProfilingWidget(QWidget* parent, Box* box);

public Q_SLOTS:
    void reposition(Box* box);

protected:
    void paintEvent(QPaintEvent *);

private:
    Box* box_;
    NodeWorker* node_worker_;

    int w_;
    int h_;
};

}

#endif // PROFILING_WIDGET_H
