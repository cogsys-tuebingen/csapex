/// HEADER
#include <csapex/model/graph_facade.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_facade.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/model/connection.h>
#include <csapex/core/settings.h>
#include <csapex/scheduling/task_generator.h>
#include <csapex/model/node_runner.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/msg/direct_connection.h>
#include <csapex/model/connectable.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/model/graph/vertex.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

GraphFacade::GraphFacade(ThreadPool &executor, SubgraphNodePtr graph, NodeFacadePtr nh, GraphFacade *parent)
    : parent_(parent), absolute_uuid_(graph->getUUID()), graph_(graph), graph_handle_(nh), executor_(executor)
{
    observe(graph->vertex_added, delegate::Delegate<void(graph::VertexPtr)>(this, &GraphFacade::nodeAddedHandler));

    observe(graph->vertex_removed, delegate::Delegate<void(graph::VertexPtr)>(this, &GraphFacade::nodeRemovedHandler));

    observe(graph->notification, notification);

    if(parent_) {
        apex_assert_hard(graph_handle_);

        AUUID parent_auuid = parent_->getAbsoluteUUID();
        if(!parent_auuid.empty()) {
            absolute_uuid_ = AUUID(UUIDProvider::makeDerivedUUID_forced(parent_auuid,
                                                                        absolute_uuid_.getFullName()));
        }
    }
}

GraphFacade::~GraphFacade()
{
    stopObserving();
}

GraphFacade* GraphFacade::getParent() const
{
    return parent_;
}

void GraphFacade::nodeAddedHandler(graph::VertexPtr vertex)
{
    NodeFacadePtr facade = vertex->getNodeFacade();
    if(facade->isGraph()) {
        createSubgraphFacade(vertex->getNodeFacade());
    }

    NodeWorkerPtr nw = facade->getNodeWorker();

    node_facades_[facade->getUUID()] = facade;

    NodeRunnerPtr runner = facade->getNodeRunner();
    apex_assert_hard(runner);
    generators_[facade->getUUID()] = runner;

    int thread_id = facade->getNodeState()->getThreadId();
    if(thread_id >= 0) {
        executor_.addToGroup(runner.get(), thread_id);
    } else {
        executor_.add(runner.get());
    }

    generator_added(runner);

    node_facade_added(facade);

    nw->notification.connect(notification);

    nw->initialize();
    nw->panic.connect(panic);
}

void GraphFacade::createSubgraphFacade(NodeFacadePtr nf)
{
    NodePtr node = nf->getNodeHandle()->getNode().lock();
    apex_assert_hard(node);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(node);
    apex_assert_hard(sub_graph);

    NodeHandle* subnh = graph_->findNodeHandle(sub_graph->getUUID());;
    apex_assert_hard(subnh == nf->getNodeHandle().get());
    GraphFacadePtr sub_graph_facade = std::make_shared<GraphFacade>(executor_, sub_graph, nf, this);
    children_[nf->getUUID()] = sub_graph_facade;

    sub_graph_facade->notification.connect(notification);

    child_added(sub_graph_facade);
}

void GraphFacade::nodeRemovedHandler(graph::VertexPtr vertex)
{
    NodeFacadePtr nh = vertex->getNodeFacade();

    TaskGeneratorPtr runner = generators_[nh->getUUID()];
    generators_.erase(nh->getUUID());
    executor_.remove(runner.get());
    generator_removed(runner);

    NodeFacadePtr facade = node_facades_[nh->getUUID()];
    node_facade_removed(facade);
    node_facades_.erase(nh->getUUID());

    if(nh->getType() == "csapex::Graph") {
        auto pos = children_.find(nh->getUUID());
        apex_assert_hard(pos != children_.end());
        child_removed(pos->second);
        children_.erase(pos);
    }
}

AUUID GraphFacade::getAbsoluteUUID() const
{
    return absolute_uuid_;
}

GraphPtr GraphFacade::getGraph()
{
    return graph_;
}

SubgraphNodePtr GraphFacade::getSubgraphNode()
{
    return std::dynamic_pointer_cast<SubgraphNode>(graph_);
}

ThreadPool* GraphFacade::getThreadPool()
{
    return &executor_;
}

NodeHandle* GraphFacade::getNodeHandle()
{
    return graph_handle_->getNodeHandle().get();
}

NodeFacadePtr GraphFacade::getNodeFacade()
{
    return graph_handle_;
}

void GraphFacade::addNode(NodeFacadePtr nh)
{
    graph_->addNode(nh);
}

GraphFacade* GraphFacade::getSubGraph(const UUID &uuid)
{
    if(uuid.empty()) {
        throw std::logic_error("cannot get subgraph for empty UUID");
    }

    if(uuid.composite()) {
        GraphFacadePtr facade = children_[uuid.rootUUID()];
        return facade->getSubGraph(uuid.nestedUUID());
    } else {
        GraphFacadePtr facade = children_[uuid];
        return facade.get();
    }
}

ConnectionPtr GraphFacade::connect(OutputPtr output, InputPtr input)
{
    auto c = DirectConnection::connect(output, input);
    graph_->addConnection(c);
    return c;
}

ConnectionPtr GraphFacade::connect(NodeHandlePtr output, int output_id,
                                   NodeHandlePtr input, int input_id)
{
    return GraphFacade::connect(output.get(), output_id, input.get(), input_id);
}

ConnectionPtr GraphFacade::connect(const UUID& output_id,
                                   NodeHandlePtr input, int input_id)
{
    NodeHandle* output = graph_->findNodeHandleForConnector(output_id);
    apex_assert_hard(output);
    OutputPtr o = output->getOutput(output_id);
    apex_assert_hard(o);
    InputPtr i = input->getInput(graph_->makeTypedUUID_forced(input->getUUID(), "in", input_id));
    if(!i) {
        throw std::logic_error(input->getUUID().getFullName() +
                               " does not have an input with the label " +
                               std::to_string(input_id));
    }
    apex_assert_hard(i);

    auto c = DirectConnection::connect(o, i);
    graph_->addConnection(c);
    return c;
}

ConnectionPtr GraphFacade::connect(NodeHandlePtr output, int output_id,
                                   const UUID& input_id)
{
    OutputPtr o = output->getOutput(graph_->makeTypedUUID_forced(output->getUUID(), "out", output_id));
    if(!o) {
        throw std::logic_error(output->getUUID().getFullName() +
                               " does not have an output with the index " +
                               std::to_string(output_id));
    }
    apex_assert_hard(o);
    NodeHandle* input = graph_->findNodeHandleForConnector(input_id);
    apex_assert_hard(input);
    InputPtr i = input->getInput(input_id);
    apex_assert_hard(i);

    auto c = DirectConnection::connect(o, i);
    graph_->addConnection(c);
    return c;
}

ConnectionPtr GraphFacade::connect(const UUID& output_id,
                                   NodeHandlePtr input, const std::string& input_id)
{
    return connect(output_id, input.get(), input_id);
}

ConnectionPtr GraphFacade::connect(const UUID& output_id,
                                   NodeHandle* input, const std::string& input_id)
{
    NodeHandle* output = graph_->findNodeHandleForConnector(output_id);
    apex_assert_hard(output);
    OutputPtr o = output->getOutput(output_id);
    apex_assert_hard(o);

    InputPtr i = nullptr;
    for(auto in : input->getExternalInputs()) {
        Input* in_ptr = in.get();
        if(in_ptr->getLabel() == input_id) {
            i = in;
            break;
        }
    }
    for(auto in : input->getSlots()) {
        if(in->getLabel() == input_id) {
            i = in;
            break;
        }
    }
    if(!i) {
        throw std::logic_error(input->getUUID().getFullName() +
                               " does not have an input with the label " +
                               input_id);
    }

    auto c = DirectConnection::connect(o, i);
    graph_->addConnection(c);
    return c;
}

ConnectionPtr GraphFacade::connect(NodeHandlePtr output, const std::string& output_id,
                                   const UUID& input_id)
{
    return connect(output.get(), output_id, input_id);
}

ConnectionPtr GraphFacade::connect(NodeHandle* output, const std::string& output_id,
                                   const UUID& input_id)
{
    OutputPtr o = nullptr;
    for(auto out : output->getExternalOutputs()) {
        if(out->getLabel() == output_id) {
            o = out;
            break;
        }
    }
    for(auto out : output->getEvents()) {
        if(out->getLabel() == output_id) {
            o = out;
            break;
        }
    }
    if(!o) {
        throw std::logic_error(output->getUUID().getFullName() +
                               " does not have an output with the label " +
                               output_id);
    }

    NodeHandle* input = graph_->findNodeHandleForConnector(input_id);
    apex_assert_hard(input);
    InputPtr i = input->getInput(input_id);
    apex_assert_hard(i);

    auto c = DirectConnection::connect(o, i);
    graph_->addConnection(c);
    return c;
}

ConnectionPtr GraphFacade::connect(NodeHandle *output, int output_id,
                                   NodeHandle *input, int input_id)
{
    OutputPtr o = output->getOutput(graph_->makeTypedUUID_forced(output->getUUID(), "out", output_id));
    if(!o) {
        throw std::logic_error(output->getUUID().getFullName() +
                               " does not have an output with the index " +
                               std::to_string(output_id));
    }
    InputPtr i = input->getInput(graph_->makeTypedUUID_forced(input->getUUID(), "in", input_id));
    if(!i) {
        throw std::logic_error(input->getUUID().getFullName() +
                               " does not have an input with the label " +
                               std::to_string(input_id));
    }

    auto c = DirectConnection::connect(o, i);
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
    OutputPtr o = nullptr;
    for(auto out : output->getExternalOutputs()) {
        if(out->getLabel() == output_id) {
            o = out;
            break;
        }
    }
    for(auto event : output->getEvents()) {
        if(event->getLabel() == output_id) {
            o = event;
            break;
        }
    }
    if(!o) {
        throw std::logic_error(output->getUUID().getFullName() +
                               " does not have an output with the label " +
                               output_id);
    }
    InputPtr i = nullptr;
    for(auto in : input->getExternalInputs()) {
        if(in->getLabel() == input_id) {
            i = in;
            break;
        }
    }
    for(auto slot : input->getSlots()) {
        if(slot->getLabel() == input_id) {
            i = slot;
            break;
        }
    }
    if(!i) {
        throw std::logic_error(input->getUUID().getFullName() +
                               " does not have an input with the label " +
                               input_id);
    }

    auto c = DirectConnection::connect(o, i);
    graph_->addConnection(c);
    return c;
}

ConnectionPtr GraphFacade::connect(const UUID& output_id, const UUID& input_id)
{
    NodeHandle* output = graph_->findNodeHandleForConnector(output_id);
    NodeHandle* input = graph_->findNodeHandleForConnector(input_id);
    apex_assert_hard(output);
    apex_assert_hard(input);
    OutputPtr o = output->getOutput(output_id);
    InputPtr i = input->getInput(input_id);
    apex_assert_hard(o);
    apex_assert_hard(i);

    auto c = DirectConnection::connect(o, i);
    graph_->addConnection(c);
    return c;
}

TaskGenerator* GraphFacade::getTaskGenerator(const UUID &uuid)
{
    return generators_.at(uuid).get();
}

void GraphFacade::clear()
{
    stop();
    graph_->clear();
    for(auto& gen: generators_) {
        generator_removed(gen.second);
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

void GraphFacade::resetActivity()
{
    bool pause = isPaused();

    pauseRequest(true);

    graph_->resetActivity();

    for(auto pair: children_) {
        GraphFacadePtr child = pair.second;
        child->resetActivity();
    }

    if(!parent_) {
        graph_->activation();
    }

    pauseRequest(pause);
}
