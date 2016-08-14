/// HEADER
#include <csapex/command/flip_sides.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex;
using namespace csapex::command;

FlipSides::FlipSides(const AUUID& parent_uuid, const UUID &node)
    : Command(parent_uuid), uuid(node)
{
}

std::string FlipSides::getType() const
{
    return "FlipSides";
}

std::string FlipSides::getDescription() const
{
    std::stringstream ss;
    ss << "flipped sides of " << uuid;
    return ss.str();
}

bool FlipSides::doExecute()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);
    apex_assert_hard(node_handle);

    bool flip = !node_handle->getNodeState()->isFlipped();
    node_handle->getNodeState()->setFlipped(flip);

    return true;
}

bool FlipSides::doUndo()
{
    return doExecute();
}

bool FlipSides::doRedo()
{
    return doExecute();
}

