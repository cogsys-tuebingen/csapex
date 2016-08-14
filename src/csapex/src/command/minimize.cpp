/// HEADER
#include <csapex/command/minimize.h>

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

Minimize::Minimize(const AUUID& parent_uuid, const UUID &node, bool mini)
    : Command(parent_uuid), uuid(node), mini(mini), executed(false)
{
}

std::string Minimize::getType() const
{
    return "Minimize";
}

std::string Minimize::getDescription() const
{
    std::stringstream ss;
    ss << ( mini ? "min" : "max" ) << "imized" << uuid;
    return ss.str();
}

bool Minimize::doExecute()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);
    apex_assert_hard(node_handle);

    bool is_mini = node_handle->getNodeState()->isMinimized();

    if(is_mini != mini) {
        node_handle->getNodeState()->setMinimized(mini);
        executed = true;
    } else {
        executed = false;
    }

    return true;
}

bool Minimize::doUndo()
{
    if(executed) {
        NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);
        apex_assert_hard(node_handle);

        node_handle->getNodeState()->setMinimized(!mini);
    }
    return true;
}

bool Minimize::doRedo()
{
    return doExecute();
}

