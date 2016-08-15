/// HEADER
#include <csapex/command/set_color.h>

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

SetColor::SetColor(const AUUID& parent_uuid, const UUID &node, int r, int g, int b)
    : Command(parent_uuid), uuid(node), r(r), g(g), b(b)
{
}

std::string SetColor::getType() const
{
    return "SetColor";
}

std::string SetColor::getDescription() const
{
    std::stringstream ss;
    ss << "set color of " << uuid << " to " << r << " " << g << " " << b;
    return ss.str();
}

bool SetColor::doExecute()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);
    apex_assert_hard(node_handle);

    node_handle->getNodeState()->getColor(r_orig, g_orig, b_orig);
    node_handle->getNodeState()->setColor(r, g, b);

    return true;
}

bool SetColor::doUndo()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);
    apex_assert_hard(node_handle);

    node_handle->getNodeState()->setColor(r_orig, g_orig, b_orig);

    return true;
}

bool SetColor::doRedo()
{
    return doExecute();
}

