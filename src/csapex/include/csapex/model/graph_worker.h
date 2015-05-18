#ifndef GRAPH_WORKER_H
#define GRAPH_WORKER_H

/// PROJECT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <boost/signals2/signal.hpp>

namespace csapex
{

class GraphWorker
{
public:
    typedef std::shared_ptr<GraphWorker> Ptr;

public:
    GraphWorker(Settings *settings, Graph* graph);

    Graph* getGraph();

    void stop();

    bool isPaused() const;
    void setPause(bool pause);

    void reset();

public:
    boost::signals2::signal<void (bool)> paused;

private:
    Graph* graph_;
    bool paused_;
};

}

#endif // GRAPH_WORKER_H
