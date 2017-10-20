/// HEADER
#include <csapex/command/rename_node.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph/graph_local.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex;
using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(RenameNode)

RenameNode::RenameNode(const AUUID& parent_uuid, const UUID &node, const std::string& new_name)
    : CommandImplementation(parent_uuid), uuid(node), new_name_(new_name)
{
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




void RenameNode::serialize(SerializationBuffer &data) const
{
    Command::serialize(data);

    data << uuid;
    data << new_name_;
    data << old_name_;
}

void RenameNode::deserialize(const SerializationBuffer& data)
{
    Command::deserialize(data);

    data >> uuid;
    data >> new_name_;
    data >> old_name_;
}
