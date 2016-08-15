/// HEADER
#include <csapex/command/rename_node.h>

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

RenameNode::RenameNode(const AUUID& parent_uuid, const UUID &node, const std::string& new_name)
    : Command(parent_uuid), uuid(node), new_name_(new_name)
{
}

std::string RenameNode::getType() const
{
    return "RenameNode";
}

std::string RenameNode::getDescription() const
{
    std::stringstream ss;
    ss << "rename node " << uuid << " from " << old_name_ << " to " << new_name_;
    return ss.str();
}

bool RenameNode::doExecute()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);
    apex_assert_hard(node_handle);

    NodeStatePtr state = node_handle->getNodeState();

    old_name_ = state->getLabel();
    state->setLabel(new_name_);

    return true;
}

bool RenameNode::doUndo()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);
    apex_assert_hard(node_handle);

    NodeStatePtr state = node_handle->getNodeState();
    state->setLabel(old_name_);

    return true;
}

bool RenameNode::doRedo()
{
    return doExecute();
}

