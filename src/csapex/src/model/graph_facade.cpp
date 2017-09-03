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

GraphFacade::GraphFacade(NodeFacadePtr nh)
    : graph_handle_(nh)
{
}

GraphFacade::~GraphFacade()
{
    stopObserving();
}

NodeHandle* GraphFacade::getNodeHandle()
{
    return graph_handle_->getNodeHandle().get();
}

NodeFacadePtr GraphFacade::getNodeFacade()
{
    return graph_handle_;
}



NodeFacadePtr GraphFacade::findNodeFacade(const UUID& uuid) const
{
    return getGraph()->findNodeFacade(uuid);
}
NodeFacadePtr GraphFacade::findNodeFacadeNoThrow(const UUID& uuid) const noexcept
{
    return getGraph()->findNodeFacadeNoThrow(uuid);
}
NodeFacadePtr GraphFacade::findNodeFacadeForConnector(const UUID &uuid) const
{
    return getGraph()->findNodeFacadeForConnector(uuid);
}
NodeFacadePtr GraphFacade::findNodeFacadeForConnectorNoThrow(const UUID &uuid) const noexcept
{
    return getGraph()->findNodeFacadeForConnectorNoThrow(uuid);
}
NodeFacadePtr GraphFacade::findNodeFacadeWithLabel(const std::string& label) const
{
    return getGraph()->findNodeFacadeWithLabel(label);
}

ConnectorPtr GraphFacade::findConnector(const UUID &uuid)
{
    return getGraph()->findConnector(uuid);
}
ConnectorPtr GraphFacade::findConnectorNoThrow(const UUID &uuid) noexcept
{
    return getGraph()->findConnectorNoThrow(uuid);
}
