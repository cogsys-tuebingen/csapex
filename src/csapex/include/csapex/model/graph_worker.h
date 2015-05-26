#ifndef GRAPH_WORKER_H
#define GRAPH_WORKER_H

/// PROJECT
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <boost/signals2/signal.hpp>
#include <unordered_map>

namespace csapex
{

class GraphWorker
{
public:
    typedef std::shared_ptr<GraphWorker> Ptr;

public:
    GraphWorker(Settings *settings, Graph* graph);

    Graph* getGraph();

    TaskGenerator* getTaskGenerator(const UUID& uuid);

    void stop();

    bool isPaused() const;
    void setPause(bool pause);

    void reset();

public:
    boost::signals2::signal<void (bool)> paused;
    boost::signals2::signal<void ()> stopped;

    boost::signals2::signal<void(TaskGeneratorPtr)> generatorAdded;
    boost::signals2::signal<void(TaskGeneratorPtr)> generatorRemoved;

private:
    Graph* graph_;

    std::unordered_map<UUID, TaskGeneratorPtr, UUID::Hasher> generators_;

    bool paused_;
};

}

#endif // GRAPH_WORKER_H
