/// HEADER
#include <csapex/model/graph_facade.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/connection.h>
#include <csapex/core/settings.h>
#include <csapex/scheduling/task_generator.h>
#include <csapex/model/node_runner.h>
#include <csapex/scheduling/executor.h>
#include <csapex/msg/bundled_connection.h>

using namespace csapex;

GraphFacade::GraphFacade(Executor &executor, Graph* graph)
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

GraphFacade::~GraphFacade()
{
    for(auto connection : connections_) {
        connection.disconnect();
    }
    connections_.clear();
}

Graph* GraphFacade::getGraph()
{
    return graph_;
}

ConnectionPtr GraphFacade::connect(Output *output, Input *input, OutputTransition *ot, InputTransition *it)
{
    auto c = BundledConnection::connect(output, input, ot, it);
    graph_->addConnection(c);
    return c;
}

TaskGenerator* GraphFacade::getTaskGenerator(const UUID &uuid)
{
    return generators_.at(uuid).get();
}

void GraphFacade::reset()
{
    stop();
    graph_->clear();
    for(auto& gen: generators_) {
        generatorRemoved(gen.second);
    }
    generators_.clear();
}

bool GraphFacade::isPaused() const
{
    return executor_.isPaused();
}

void GraphFacade::pauseRequest(bool pause)
{
    if(executor_.isPaused() == pause) {
        return;
    }

    executor_.setPause(pause);

    paused(pause);
}


void GraphFacade::stop()
{
    for(NodeWorker* nw : graph_->getAllNodeWorkers()) {
        nw->stop();
    }

    executor_.stop();

    stopped();    
}

void GraphFacade::clearBlock()
{
    executor_.clear();
}
