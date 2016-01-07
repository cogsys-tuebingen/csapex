/// HEADER
#include <csapex/command/group_nodes.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex::command;

GroupNodes::GroupNodes(const std::vector<UUID> &nodes)
    : uuids(nodes)
{
}

std::string GroupNodes::getType() const
{
    return "GroupNodes";
}

std::string GroupNodes::getDescription() const
{
    return "create a sub graph";
}

bool GroupNodes::doExecute()
{
    // let S be the set of all selected nodes
    // 1. create graph
    // 2. remember all connections
    // 3. for each s in S:
    //     clone node state
    //     delete nodes in S
    //     create nodes in sub graph
    //     restore node state
    // 4. make inputs available, which lead out of S
    // 5. restore connections inside the sub graph
    // 6. create additional connections
    return true;
}

bool GroupNodes::doUndo()
{
    return false;
}

bool GroupNodes::doRedo()
{
    return doExecute();
}

