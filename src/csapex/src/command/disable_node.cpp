/// HEADER
#include <csapex/command/disable_node.h>

/// COMPONENT
#include <csapex/command/delete_connection.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/node_state.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM

using namespace csapex;
using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(DisableNode)

DisableNode::DisableNode(const AUUID& parent_uuid, const UUID& uuid, bool disable)
    : CommandImplementation(parent_uuid), uuid(uuid), disable_(disable)
{
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


void DisableNode::serialize(SerializationBuffer &data) const
{
    Command::serialize(data);

    data << uuid;
    data << disable_;
}

void DisableNode::deserialize(const SerializationBuffer& data)
{
    Command::deserialize(data);

    data >> uuid;
    data >> disable_;
}
