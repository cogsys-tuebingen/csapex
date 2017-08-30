/// HEADER
#include <csapex/model/graph_facade.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_facade_local.h>
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

GraphFacade::GraphFacade(ThreadPool &executor, GraphPtr graph, SubgraphNodePtr graph_node, NodeFacadePtr nh)
    : absolute_uuid_(graph_node->getUUID()), graph_(graph), graph_node_(graph_node), graph_handle_(nh), executor_(executor)
{
    observe(graph->vertex_added, delegate::Delegate<void(graph::VertexPtr)>(this, &GraphFacade::nodeAddedHandler));

    observe(graph->vertex_removed, delegate::Delegate<void(graph::VertexPtr)>(this, &GraphFacade::nodeRemovedHandler));

    observe(graph->notification, notification);
}

GraphFacade::~GraphFacade()
{
    stopObserving();
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
    return std::dynamic_pointer_cast<SubgraphNode>(graph_node_);
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


void GraphFacade::clear()
{
    stop();
    graph_->clear();
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
