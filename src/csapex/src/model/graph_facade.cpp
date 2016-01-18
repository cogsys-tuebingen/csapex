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
#include <csapex/scheduling/thread_pool.h>
#include <csapex/msg/bundled_connection.h>
#include <csapex/model/connectable.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>

using namespace csapex;

GraphFacade::GraphFacade(ThreadPool &executor, Graph* graph)
    : graph_(graph), executor_(executor)
{
    connections_.push_back(graph->nodeAdded.connect(
                               delegate::Delegate<void(NodeHandlePtr)>(this, &GraphFacade::nodeAddedHandler)));

    connections_.push_back(graph->nodeRemoved.connect(
                               delegate::Delegate<void(NodeHandlePtr)>(this, &GraphFacade::nodeRemovedHandler)));
}

GraphFacade::~GraphFacade()
{
    for(auto& connection : connections_) {
        connection.disconnect();
    }
    connections_.clear();
}


void GraphFacade::nodeAddedHandler(NodeHandlePtr nh) {
    if(nh->getType() == "csapex::Graph") {
        NodePtr node = nh->getNode().lock();
        apex_assert_hard(node);
        GraphPtr sub_graph = std::dynamic_pointer_cast<Graph>(node);
        apex_assert_hard(sub_graph);

        GraphFacadePtr sub_graph_facade = std::make_shared<GraphFacade>(executor_, sub_graph.get());
        children_[nh->getUUID()] = sub_graph_facade;

        childAdded(sub_graph_facade);
    }

    NodeWorkerPtr nw = std::make_shared<NodeWorker>(nh);
    node_workers_[nh.get()] = nw;

    TaskGeneratorPtr runner = std::make_shared<NodeRunner>(nw);
    generators_[nh->getUUID()] = runner;
    executor_.add(runner.get());
    generatorAdded(runner);

    nodeAdded(nh);
    nodeWorkerAdded(nw);

    nw->checkParameters();
    nw->panic.connect(panic);

}

void GraphFacade::nodeRemovedHandler(NodeHandlePtr nh)
{
    TaskGeneratorPtr runner = generators_[nh->getUUID()];
    generators_.erase(nh->getUUID());
    executor_.remove(runner.get());
    generatorRemoved(runner);

    NodeWorkerPtr nw = node_workers_[nh.get()];
    nodeWorkerRemoved(nw);
    node_workers_.erase(nh.get());
    nodeRemoved(nh);

    if(nh->getType() == "csapex::Graph") {
        auto pos = children_.find(nh->getUUID());
        childRemoved(pos->second);
        children_.erase(pos);
    }

}

Graph* GraphFacade::getGraph()
{
    return graph_;
}

ThreadPool* GraphFacade::getThreadPool()
{
    return &executor_;
}

NodeWorkerPtr GraphFacade::getNodeWorker(const NodeHandle *handle)
{
    return node_workers_[handle];
}

void GraphFacade::addNode(NodeHandlePtr nh)
{
    graph_->addNode(nh);
}

GraphFacade* GraphFacade::getSubGraph(const UUID &uuid)
{
    GraphFacadePtr facade = children_[uuid];
    return facade.get();
}

ConnectionPtr GraphFacade::connect(OutputPtr output, InputPtr input, OutputTransition *ot, InputTransition *it)
{
    return GraphFacade::connect(output.get(), input.get(), ot, it);
}

ConnectionPtr GraphFacade::connect(Output *output, Input *input, OutputTransition *ot, InputTransition *it)
{
    auto c = BundledConnection::connect(output, input, ot, it);
    graph_->addConnection(c);
    return c;
}

ConnectionPtr GraphFacade::connect(NodeHandlePtr output, int output_id,
                                   NodeHandlePtr input, int input_id)
{
    return GraphFacade::connect(output.get(), output_id, input.get(), input_id);
}

ConnectionPtr GraphFacade::connect(NodeHandle *output, int output_id,
                                   NodeHandle *input, int input_id)
{
    Output* o = output->getOutput(graph_->makeConnectableUUID_forced(output->getUUID(), "out", output_id));
    if(!o) {
        throw std::logic_error(output->getUUID().getFullName() +
                               " does not have an output with the index " +
                               std::to_string(output_id));
    }
    Input* i = input->getInput(graph_->makeConnectableUUID_forced(input->getUUID(), "in", input_id));
    if(!i) {
        throw std::logic_error(input->getUUID().getFullName() +
                               " does not have an input with the label " +
                               std::to_string(input_id));
    }

    auto c = BundledConnection::connect(o, i, output->getOutputTransition(), input->getInputTransition());
    graph_->addConnection(c);
    return c;
}

ConnectionPtr GraphFacade::connect(NodeHandlePtr output, const std::string& output_id,
                                   NodeHandlePtr input, const std::string& input_id)
{
    return GraphFacade::connect(output.get(), output_id, input.get(), input_id);
}

ConnectionPtr GraphFacade::connect(NodeHandle *output, const std::string& output_id,
                                   NodeHandle *input, const std::string& input_id)
{
    Output* o = nullptr;
    for(auto out : output->getAllOutputs()) {
        Output* out_ptr = out.get();
        if(out_ptr->getLabel() == output_id) {
            o = out_ptr;
            break;
        }
    }
    if(!o) {
        throw std::logic_error(output->getUUID().getFullName() +
                               " does not have an output with the label " +
                               output_id);
    }
    Input* i = nullptr;
    for(auto in : input->getAllInputs()) {
        Input* in_ptr = in.get();
        if(in_ptr->getLabel() == input_id) {
            i = in_ptr;
            break;
        }
    }
    if(!i) {
        throw std::logic_error(input->getUUID().getFullName() +
                               " does not have an input with the label " +
                               input_id);
    }
    auto c = BundledConnection::connect(o, i, output->getOutputTransition(), input->getInputTransition());
    graph_->addConnection(c);
    return c;
}

ConnectionPtr GraphFacade::connect(const UUID& output_id, const UUID& input_id)
{
    NodeHandle* output = graph_->findNodeHandleForConnector(output_id);
    NodeHandle* input = graph_->findNodeHandleForConnector(input_id);
    apex_assert_hard(output);
    apex_assert_hard(input);
    Output* o = output->getOutput(output_id);
    Input* i = input->getInput(input_id);
    apex_assert_hard(o);
    apex_assert_hard(i);
    auto c = BundledConnection::connect(o, i, output->getOutputTransition(), input->getInputTransition());
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
    for(NodeHandle* nw : graph_->getAllNodeHandles()) {
        nw->stop();
    }

    executor_.stop();

    stopped();
}

void GraphFacade::clearBlock()
{
    executor_.clear();
}
