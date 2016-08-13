/// HEADER
#include <csapex/command/disable_node.h>

/// COMPONENT
#include <csapex/command/delete_connection.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/graph.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/node_state.h>

/// SYSTEM

using namespace csapex;
using namespace csapex::command;

DisableNode::DisableNode(const AUUID& parent_uuid, const UUID& uuid, bool disable)
    : Command(parent_uuid), uuid(uuid), disable_(disable)
{
}

std::string DisableNode::getType() const
{
    if(disable_) {
        return "DisableNode";
    } else {
        return "EnableNode";
    }
}

std::string DisableNode::getDescription() const
{
    if(disable_) {
        return std::string("disable node ") + uuid.getFullName();
    } else {
        return std::string("enable node ") + uuid.getFullName();
    }
}


bool DisableNode::doExecute()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);

//    node_handle->setProcessingEnabled(!disable_);
    node_handle->getNodeState()->setEnabled(!disable_);

    return true;
}

bool DisableNode::doUndo()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);

//    node_handle->setProcessingEnabled(disable_);
    node_handle->getNodeState()->setEnabled(disable_);

    return true;
}

bool DisableNode::doRedo()
{
    return doExecute();
}
