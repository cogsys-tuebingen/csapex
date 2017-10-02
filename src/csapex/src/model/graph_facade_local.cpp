/// HEADER
#include <csapex/model/graph_facade_local.h>

/// PROJECT
#include <csapex/model/node_facade_local.h>
#include <csapex/model/graph_facade_local.h>
#include <csapex/model/graph/graph_local.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_runner.h>
#include <csapex/model/node_handle.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/model/node_worker.h>
#include <csapex/msg/direct_connection.h>
#include <csapex/model/connectable.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>

using namespace csapex;

GraphFacadeLocal::GraphFacadeLocal(ThreadPool &executor, GraphLocalPtr graph, SubgraphNodePtr graph_node, NodeFacadePtr nh, GraphFacadeLocal *parent)
    : GraphFacade(nh),
      absolute_uuid_(graph_node->getUUID()),
      parent_(parent),
      executor_(executor),
      graph_(graph),
      graph_node_(graph_node)
{
    observe(graph->vertex_added, delegate::Delegate<void(graph::VertexPtr)>(this, &GraphFacadeLocal::nodeAddedHandler));
    observe(graph->vertex_removed, delegate::Delegate<void(graph::VertexPtr)>(this, &GraphFacadeLocal::nodeRemovedHandler));
    observe(graph->notification, notification);

    observe(graph_node_->forwardingAdded, forwardingAdded);
    observe(graph_node_->forwardingRemoved, forwardingRemoved);

    if(parent_) {
        // TODO: refactor!
        apex_assert_hard(graph_handle_);

        AUUID parent_auuid = parent_->getAbsoluteUUID();
        if(!parent_auuid.empty()) {
            absolute_uuid_ = AUUID(UUIDProvider::makeDerivedUUID_forced(parent_auuid,
                                                                        absolute_uuid_.getFullName()));
        }
    }
}

AUUID GraphFacadeLocal::getAbsoluteUUID() const
{
    return absolute_uuid_;
}

GraphFacade *GraphFacadeLocal::getParent() const
{
    return parent_;
}

GraphFacade* GraphFacadeLocal::getSubGraph(const UUID &uuid)
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

GraphFacadeLocalPtr GraphFacadeLocal::getLocalSubGraph(const UUID &uuid)
{
    if(uuid.empty()) {
        throw std::logic_error("cannot get subgraph for empty UUID");
    }

    if(uuid.composite()) {
        GraphFacadeLocalPtr facade = children_[uuid.rootUUID()];
        return facade->getLocalSubGraph(uuid.nestedUUID());
    } else {
        return children_[uuid];
    }
}

SubgraphNodePtr GraphFacadeLocal::getSubgraphNode()
{
    return std::dynamic_pointer_cast<SubgraphNode>(graph_node_);
}


GraphPtr GraphFacadeLocal::getGraph() const
{
    return graph_;
}

GraphLocalPtr GraphFacadeLocal::getLocalGraph() const
{
    return graph_;
}


GraphFacadeLocal *GraphFacadeLocal::getLocalParent() const
{
    return parent_;
}
ThreadPool* GraphFacadeLocal::getThreadPool()
{
    return &executor_;
}


TaskGenerator* GraphFacadeLocal::getTaskGenerator(const UUID &uuid)
{
    return generators_.at(uuid).get();
}

void GraphFacadeLocal::addNode(NodeFacadePtr nh)
{
    graph_->addNode(nh);
}

void GraphFacadeLocal::clear()
{
    stop();
    graph_->clear();

    generators_.clear();
}


void GraphFacadeLocal::stop()
{
    for(NodeHandle* nw : graph_->getAllNodeHandles()) {
        nw->stop();
    }

    executor_.stop();

    stopped();
}


ConnectionPtr GraphFacadeLocal::connect(OutputPtr output, InputPtr input)
{
    auto c = DirectConnection::connect(output, input);
    graph_->addConnection(c);
    return c;
}

ConnectionPtr GraphFacadeLocal::connect(NodeHandlePtr output, int output_id,
                                   NodeHandlePtr input, int input_id)
{
    return GraphFacadeLocal::connect(output.get(), output_id, input.get(), input_id);
}

ConnectionPtr GraphFacadeLocal::connect(const UUID& output_id,
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

ConnectionPtr GraphFacadeLocal::connect(NodeHandlePtr output, int output_id,
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

ConnectionPtr GraphFacadeLocal::connect(const UUID& output_id,
                                   NodeHandlePtr input, const std::string& input_id)
{
    return connect(output_id, input.get(), input_id);
}

ConnectionPtr GraphFacadeLocal::connect(const UUID& output_id,
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

ConnectionPtr GraphFacadeLocal::connect(NodeHandlePtr output, const std::string& output_id,
                                   const UUID& input_id)
{
    return connect(output.get(), output_id, input_id);
}

ConnectionPtr GraphFacadeLocal::connect(NodeHandle* output, const std::string& output_id,
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

ConnectionPtr GraphFacadeLocal::connect(NodeHandle *output, int output_id,
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

ConnectionPtr GraphFacadeLocal::connect(NodeHandlePtr output, const std::string& output_id,
                                   NodeHandlePtr input, const std::string& input_id)
{
    return GraphFacadeLocal::connect(output.get(), output_id, input.get(), input_id);
}

ConnectionPtr GraphFacadeLocal::connect(NodeHandle *output, const std::string& output_id,
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

ConnectionPtr GraphFacadeLocal::connect(const UUID& output_id, const UUID& input_id)
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


ConnectionPtr GraphFacadeLocal::connect(NodeFacade* output, const std::string& output_name, NodeFacade* input, const std::string& input_name)
{
    return connect(output->getNodeHandle(), output_name, input->getNodeHandle(), input_name);
}

ConnectionPtr GraphFacadeLocal::connect(NodeFacadePtr output, const std::string& output_name, NodeFacadePtr input, const std::string& input_name)
{
    return connect(output->getNodeHandle(), output_name, input->getNodeHandle(), input_name);
}
ConnectionPtr GraphFacadeLocal::connect(NodeFacade* output, const std::string& output_name, const UUID& input_id)
{
    return connect(output->getNodeHandle(), output_name, input_id);
}
ConnectionPtr GraphFacadeLocal::connect(NodeFacadePtr output, const std::string& output_name, const UUID& input_id)
{
    return connect(output->getNodeHandle(), output_name, input_id);
}
ConnectionPtr GraphFacadeLocal::connect(const UUID& output_id, NodeFacade* input, const std::string& input_name)
{
    return connect(output_id, input->getNodeHandle(), input_name);
}
ConnectionPtr GraphFacadeLocal::connect(const UUID& output_id, NodeFacadePtr input, const std::string& input_name)
{
    return connect(output_id, input->getNodeHandle(), input_name);
}
ConnectionPtr GraphFacadeLocal::connect(const UUID& output_id, NodeFacadePtr input, int input_id)
{
    return connect(output_id, input->getNodeHandle(), input_id);
}
ConnectionPtr GraphFacadeLocal::connect(NodeFacadePtr output, int output_id, const UUID& input_id)
{
    return connect(output->getNodeHandle(), output_id, input_id);
}

ConnectionPtr GraphFacadeLocal::connect(NodeFacade* output, int output_id, NodeFacade* input, int input_id)
{
    return connect(output->getNodeHandle(), output_id, input->getNodeHandle(), input_id);
}
ConnectionPtr GraphFacadeLocal::connect(NodeFacadePtr output, int output_id, NodeFacadePtr input, int input_id)
{
    return connect(output->getNodeHandle(), output_id, input->getNodeHandle(), input_id);
}
void GraphFacadeLocal::nodeAddedHandler(graph::VertexPtr vertex)
{
    NodeFacadeLocalPtr facade = std::dynamic_pointer_cast<NodeFacadeLocal>(vertex->getNodeFacade());
    apex_assert_hard(facade);
    if(facade->isGraph()) {
        createSubgraphFacade(facade);
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


    nw->notification.connect(notification);

    nw->initialize();
    nw->panic.connect(panic);

    node_facade_added(facade);
}

void GraphFacadeLocal::nodeRemovedHandler(graph::VertexPtr vertex)
{
    NodeFacadePtr nh = vertex->getNodeFacade();

    TaskGeneratorPtr runner = generators_[nh->getUUID()];
    generators_.erase(nh->getUUID());
    executor_.remove(runner.get());

    NodeFacadePtr facade = node_facades_[nh->getUUID()];
    node_facade_removed(facade);
    node_facades_.erase(nh->getUUID());

    if(nh->isGraph()) {
        auto pos = children_.find(nh->getUUID());
        apex_assert_hard(pos != children_.end());
        child_removed(pos->second);
        children_.erase(pos);
    }
}

void GraphFacadeLocal::createSubgraphFacade(NodeFacadePtr nf)
{
    NodeFacadeLocalPtr local_facade = std::dynamic_pointer_cast<NodeFacadeLocal>(nf);
    apex_assert_hard(local_facade);

    NodePtr node = local_facade->getNode();
    apex_assert_hard(node);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(node);
    apex_assert_hard(sub_graph);

    NodeHandle* subnh = graph_->findNodeHandle(local_facade->getUUID());
    apex_assert_hard(subnh == local_facade->getNodeHandle().get());

    GraphLocalPtr graph_local = sub_graph->getLocalGraph();

    GraphFacadeLocalPtr sub_graph_facade = std::make_shared<GraphFacadeLocal>(executor_, graph_local, sub_graph, local_facade, this);
    children_[local_facade->getUUID()] = sub_graph_facade;

    observe(sub_graph_facade->notification, notification);
    observe(sub_graph_facade->node_facade_added, child_node_facade_added);
    observe(sub_graph_facade->node_facade_removed, child_node_facade_removed);
    observe(sub_graph_facade->child_node_facade_added, child_node_facade_added);
    observe(sub_graph_facade->child_node_facade_removed, child_node_facade_removed);

    child_added(sub_graph_facade);
}

void GraphFacadeLocal::clearBlock()
{
    executor_.clear();
}

void GraphFacadeLocal::resetActivity()
{
    bool pause = isPaused();

    pauseRequest(true);

    graph_->resetActivity();

    for(auto pair: children_) {
        GraphFacadePtr child = pair.second;
        child->resetActivity();
    }

    if(!parent_) {
        graph_node_->activation();
    }

    pauseRequest(pause);
}


bool GraphFacadeLocal::isPaused() const
{
    return executor_.isPaused();
}

void GraphFacadeLocal::pauseRequest(bool pause)
{
    if(executor_.isPaused() == pause) {
        return;
    }

    executor_.setPause(pause);

    paused(pause);
}

std::string GraphFacadeLocal::makeStatusString()
{
    return graph_node_->makeStatusString();
}
