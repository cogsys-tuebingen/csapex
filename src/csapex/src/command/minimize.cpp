/// HEADER
#include <csapex/command/minimize.h>

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

CSAPEX_REGISTER_COMMAND_SERIALIZER(Minimize)

Minimize::Minimize(const AUUID& parent_uuid, const UUID &node, bool mini)
    : CommandImplementation(parent_uuid), uuid(node), mini(mini), executed(false)
{
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



void Minimize::serialize(SerializationBuffer &data) const
{
    Command::serialize(data);

    data << uuid;
    data << mini;
    data << executed;
}

void Minimize::deserialize(SerializationBuffer& data)
{
    Command::deserialize(data);

    data >> uuid;
    data >> mini;
    data >> executed;
}
