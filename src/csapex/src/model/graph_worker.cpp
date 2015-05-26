/// HEADER
#include <csapex/model/graph_worker.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/connection.h>
#include <csapex/core/settings.h>
#include <csapex/scheduling/task_generator.h>
#include <csapex/model/node_runner.h>

/// SYSTEM
#include <QTimer>

using namespace csapex;

GraphWorker::GraphWorker(Settings* /*settings*/, Graph* graph)
    : graph_(graph), paused_(false)
{
    graph->nodeAdded.connect([this](NodeWorkerPtr n) {
        TaskGeneratorPtr runner = std::make_shared<NodeRunner>(n);
        generators_[n->getUUID()] = runner;
        generatorAdded(runner);
    });
    graph->nodeRemoved.connect([this](NodeWorkerPtr n) {
        TaskGeneratorPtr runner = generators_[n->getUUID()];
        generators_.erase(n->getUUID());
        generatorRemoved(runner);
    });
}

Graph* GraphWorker::getGraph()
{
    return graph_;
}

TaskGenerator* GraphWorker::getTaskGenerator(const UUID &uuid)
{
    return generators_.at(uuid).get();
}

void GraphWorker::reset()
{
    stop();

    graph_->uuids_.clear();
    graph_->connections_.clear();
}

bool GraphWorker::isPaused() const
{
    return paused_;
}

void GraphWorker::setPause(bool pause)
{
    if(pause == isPaused()) {
        return;
    }

    paused_ = pause;

    paused(pause);
}


void GraphWorker::stop()
{
    stopped();

    graph_->nodes_.clear();
}
