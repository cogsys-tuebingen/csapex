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
#include <csapex/scheduling/executor.h>

using namespace csapex;

GraphWorker::GraphWorker(Executor &executor, Graph* graph)
    : graph_(graph), executor_(executor)
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
    return executor_.isPaused();
}

void GraphWorker::pauseRequest(bool pause)
{
    if(executor_.isPaused() == pause) {
        return;
    }

    executor_.setPause(pause);

    paused(pause);
}


void GraphWorker::stop()
{
    stopped();

    graph_->nodes_.clear();
}

void GraphWorker::clearBlock()
{
    // TODO: move to executor completely?
    for(NodeWorker* nw : graph_->getAllNodeWorkers()) {
        nw->reset();
    }
}
