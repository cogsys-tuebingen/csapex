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

NodeFacadeLocalPtr GraphFacadeLocal::getLocalNodeFacade() const
{
    return std::dynamic_pointer_cast<NodeFacadeLocal>(graph_handle_);
}

ThreadPool* GraphFacadeLocal::getThreadPool()
{
    return &executor_;
}


TaskGenerator* GraphFacadeLocal::getTaskGenerator(const UUID &uuid)
{
    return generators_.at(uuid).get();
}

void GraphFacadeLocal::addNode(NodeFacadeLocalPtr nh)
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
    return connect(output.get(), output_id, input.get(), input_id);
}

ConnectionPtr GraphFacadeLocal::connect(const UUID& output_id,
                                        NodeHandlePtr input, int input_id)
{
    OutputPtr o = getOutput(output_id);
    InputPtr i = getInput(getInputUUID(input.get(), input_id));
    return connect(o, i);
}

ConnectionPtr GraphFacadeLocal::connect(NodeHandlePtr output, int output_id,
                                        const UUID& input_id)
{
    OutputPtr o = getOutput(getOutputUUID(output.get(), output_id));
    InputPtr i = getInput(input_id);
    return connect(o, i);
}

ConnectionPtr GraphFacadeLocal::connect(const UUID& output_id,
                                        NodeHandlePtr input, const std::string& input_id)
{
    return connect(output_id, input.get(), input_id);
}

ConnectionPtr GraphFacadeLocal::connect(const UUID& output_id,
                                        NodeHandle* input, const std::string& input_id)
{
    OutputPtr o = getOutput(output_id);
    InputPtr i = getInput(getInputUUID(input, input_id));
    return connect(o, i);
}

ConnectionPtr GraphFacadeLocal::connect(NodeHandlePtr output, const std::string& output_id,
                                        const UUID& input_id)
{
    return connect(output.get(), output_id, input_id);
}

ConnectionPtr GraphFacadeLocal::connect(NodeHandle* output, const std::string& output_id,
                                        const UUID& input_id)
{
    OutputPtr o = getOutput(getOutputUUID(output, output_id));
    InputPtr i = getInput(input_id);
    return connect(o, i);
}

ConnectionPtr GraphFacadeLocal::connect(NodeHandle *output, int output_id,
                                        NodeHandle *input, int input_id)
{
    OutputPtr o = getOutput(getOutputUUID(output, output_id));
    InputPtr i = getInput(getInputUUID(input, input_id));
    return connect(o, i);
}

ConnectionPtr GraphFacadeLocal::connect(NodeHandlePtr output, const std::string& output_id,
                                        NodeHandlePtr input, const std::string& input_id)
{
    return connect(output.get(), output_id, input.get(), input_id);
}

ConnectionPtr GraphFacadeLocal::connect(NodeHandle *output, const std::string& output_name,
                                        NodeHandle *input, const std::string& input_name)
{
    OutputPtr o = getOutput(getOutputUUID(output, output_name));
    InputPtr i = getInput(getInputUUID(input, input_name));
    return connect(o, i);
}

ConnectionPtr GraphFacadeLocal::connect(const UUID& output_id, const UUID& input_id)
{
    OutputPtr o = getOutput(output_id);
    InputPtr i = getInput(input_id);
    return connect(o, i);
}


ConnectionPtr GraphFacadeLocal::connect(const UUID& output_id, NodeFacade* input, const std::string& input_name)
{
    UUID i = getInputUUID(input, input_name);
    return connect(output_id, i);
}
ConnectionPtr GraphFacadeLocal::connect(const UUID& output_id, NodeFacadePtr input, const std::string& input_name)
{
    return connect(output_id, input.get(), input_name);
}
ConnectionPtr GraphFacadeLocal::connect(const UUID& output_id, NodeFacadePtr input, int input_id)
{
    UUID i = getInputUUID(input.get(), input_id);
    return connect(output_id, i);
}

ConnectionPtr GraphFacadeLocal::connect(NodeFacade* output, const std::string& output_name, NodeFacade* input, const std::string& input_name)
{
    UUID o = getOutputUUID(output, output_name);
    UUID i = getInputUUID(input, input_name);
    return connect(o, i);
}
ConnectionPtr GraphFacadeLocal::connect(NodeFacadePtr output, const std::string& output_name, NodeFacadePtr input, const std::string& input_name)
{
    return connect(output.get(), output_name, input.get(), input_name);
}
ConnectionPtr GraphFacadeLocal::connect(NodeFacade* output, const std::string& output_name, const UUID& input_id)
{
    UUID o = getOutputUUID(output, output_name);
    return connect(o, input_id);
}
ConnectionPtr GraphFacadeLocal::connect(NodeFacadePtr output, const std::string& output_name, const UUID& input_id)
{
    return connect(output.get(), output_name, input_id);
}
ConnectionPtr GraphFacadeLocal::connect(NodeFacadePtr output, int output_id, const UUID& input_id)
{
    UUID o = getOutputUUID(output.get(), output_id);
    return connect(o, input_id);
}
ConnectionPtr GraphFacadeLocal::connect(NodeFacade* output, int output_id, NodeFacade* input, int input_id)
{
    UUID o = getOutputUUID(output, output_id);
    UUID i = getInputUUID(input, input_id);
    return connect(o, i);
}
ConnectionPtr GraphFacadeLocal::connect(NodeFacadePtr output, int output_id, NodeFacadePtr input, int input_id)
{
    UUID o = getOutputUUID(output.get(), output_id);
    UUID i = getInputUUID(input.get(), input_id);
    return connect(o, i);
}



OutputPtr GraphFacadeLocal::getOutput(const UUID& uuid)
{
    OutputPtr o = std::dynamic_pointer_cast<Output>(getConnectable(uuid));
    apex_assert_hard(o);
    return o;
}

InputPtr GraphFacadeLocal::getInput(const UUID& uuid)
{
    InputPtr i = std::dynamic_pointer_cast<Input>(getConnectable(uuid));
    apex_assert_hard(i);
    return i;
}


ConnectablePtr GraphFacadeLocal::getConnectable(const UUID& uuid)
{
    NodeHandle* node = graph_->findNodeHandleForConnector(uuid);
    apex_assert_hard(node);
    return node->getConnector(uuid);
}


UUID GraphFacadeLocal::getOutputUUID(NodeFacade* node, const std::string& label)
{
    for(const ConnectorDescription& out : node->getExternalOutputs()) {
        if(out.label == label) {
            return out.id;
        }
    }
    for(const ConnectorDescription& event : node->getEvents()) {
        if(event.label == label) {
            return event.id;
        }
    }
    throw std::logic_error(node->getUUID().getFullName() +
                           " does not have an output with the label " +
                           label);
}

UUID GraphFacadeLocal::getInputUUID(NodeFacade* node, const std::string& label)
{
    for(const ConnectorDescription& in : node->getExternalInputs()) {
        if(in.label == label) {
            return in.id;
        }
    }
    for(const ConnectorDescription& slot : node->getSlots()) {
        if(slot.label == label) {
            return slot.id;
        }
    }
    throw std::logic_error(node->getUUID().getFullName() +
                           " does not have an input with the label " +
                           label);
}


UUID GraphFacadeLocal::getOutputUUID(NodeHandle* node, const std::string& label)
{
    for(const OutputPtr& out : node->getExternalOutputs()) {
        if(out->getLabel() == label) {
            return out->getUUID();
        }
    }
    for(const EventPtr& event : node->getEvents()) {
        if(event->getLabel() == label) {
            return event->getUUID();
        }
    }
    throw std::logic_error(node->getUUID().getFullName() +
                           " does not have an output with the label " +
                           label);
}

UUID GraphFacadeLocal::getInputUUID(NodeHandle* node, const std::string& label)
{
    for(const InputPtr& in : node->getExternalInputs()) {
        if(in->getLabel() == label) {
            return in->getUUID();
        }
    }
    for(const SlotPtr& slot : node->getSlots()) {
        if(slot->getLabel() == label) {
            return slot->getUUID();
        }
    }
    throw std::logic_error(node->getUUID().getFullName() +
                           " does not have an input with the label " +
                           label);
}


template <class Container>
UUID GraphFacadeLocal::getOutputUUID(Container* node, int id)
{
    return graph_->makeTypedUUID_forced(node->getUUID(), "out", id);
}
template <class Container>
UUID GraphFacadeLocal::getInputUUID(Container* node, int id)
{
    return graph_->makeTypedUUID_forced(node->getUUID(), "in", id);
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


//    nw->notification.connect(notification);
    vertex->getNodeFacade()->notification.connect(notification);

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

std::vector<ConnectionInformation> GraphFacadeLocal::enumerateAllConnections() const
{
    std::vector<ConnectionInformation> result;
    auto connections = graph_->getConnections();
    result.reserve(connections.size());
    for(const ConnectionPtr& c : connections) {
        result.push_back(c->getDescription());
    }
    return result;
}
