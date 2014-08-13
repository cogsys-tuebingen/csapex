#ifndef GRAPH_WORKER_H
#define GRAPH_WORKER_H

/// PROJECT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QObject>

class QTimer;

namespace csapex
{

class GraphWorker : public QObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<GraphWorker> Ptr;

public:
    GraphWorker(Graph* graph);

    Graph* getGraph();

    void stop();

    bool isPaused() const;
    void setPause(bool pause);

public Q_SLOTS:
    void tick();
    void reset();

Q_SIGNALS:
    void paused(bool);

private:
    Graph* graph_;

    QTimer* timer_;
};

}

#endif // GRAPH_WORKER_H
