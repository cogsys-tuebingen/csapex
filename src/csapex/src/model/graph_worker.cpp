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
    connections_.push_back(graph->nodeAdded.connect([this](NodeWorkerPtr n) {
        TaskGeneratorPtr runner = std::make_shared<NodeRunner>(n);
        generators_[n->getUUID()] = runner;
        executor_.add(runner.get());
        generatorAdded(runner);
    }));
    connections_.push_back(graph->nodeRemoved.connect([this](NodeWorkerPtr n) {
        TaskGeneratorPtr runner = generators_[n->getUUID()];
        generators_.erase(n->getUUID());
        executor_.remove(runner.get());
        generatorRemoved(runner);
    }));
}

GraphWorker::~GraphWorker()
{
    for(auto connection : connections_) {
        connection.disconnect();
    }
    connections_.clear();
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
    graph_->clear();
    for(auto& gen: generators_) {
        generatorRemoved(gen.second);
    }
    generators_.clear();
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
    for(NodeWorker* nw : graph_->getAllNodeWorkers()) {
        nw->stop();
    }

    executor_.stop();

    stopped();    
}

void GraphWorker::clearBlock()
{
    executor_.clear();
}
