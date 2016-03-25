/// HEADER
#include <csapex/command/ungroup_nodes.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/add_node.h>
#include <csapex/command/add_msg_connection.h>
#include <csapex/command/paste_graph.h>
#include <csapex/command/pass_out_connector.h>
#include <csapex/model/graph_facade.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>
#include <csapex/core/graphio.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex::command;

UngroupNodes::UngroupNodes(const AUUID& parent_uuid, const UUID &uuid)
    : Meta(parent_uuid, "UngroupNodes"), uuid(uuid)
{
}

std::string UngroupNodes::getType() const
{
    return "UngroupNodes";
}

std::string UngroupNodes::getDescription() const
{
    return "inline a sub graph";
}

bool UngroupNodes::doExecute()
{
    Graph* graph = getGraph();

    CommandPtr del = std::make_shared<command::DeleteNode>(getGraphFacade()->getAbsoluteUUID(), uuid);
    executeCommand(del);
    add(del);

    return true;
}

bool UngroupNodes::doUndo()
{
    return Meta::doUndo();
}

bool UngroupNodes::doRedo()
{
    // problem: uses the graph, not the subgraph!
    locked = false;
    clear();
    return doExecute();
}

