/// HEADER
#include <csapex/command/set_execution_mode.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex;
using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(SetExecutionMode)

SetExecutionMode::SetExecutionMode(const AUUID& parent_uuid, const UUID& node, ExecutionMode mode) : CommandImplementation(parent_uuid), uuid(node), mode(mode)
{
}

std::string SetExecutionMode::getDescription() const
{
    std::stringstream ss;
    ss << "set the execution mode of " << uuid << " to ";
    switch (mode) {
        case ExecutionMode::PIPELINING:
            ss << "PIPELINING";
            break;
        case ExecutionMode::SEQUENTIAL:
            ss << "SEQUENTIAL";
            break;
    }

    return ss.str();
}

bool SetExecutionMode::doExecute()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);
    apex_assert_hard(node_handle);

    NodeStatePtr state = node_handle->getNodeState();
    was_mode = state->getExecutionMode();

    state->setExecutionMode(mode);

    return true;
}

bool SetExecutionMode::doUndo()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);
    apex_assert_hard(node_handle);

    NodeStatePtr state = node_handle->getNodeState();

    state->setExecutionMode(was_mode);

    return true;
}

bool SetExecutionMode::doRedo()
{
    return doExecute();
}

void SetExecutionMode::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    Command::serialize(data, version);

    data << uuid;
    data << was_mode;
    data << mode;
}

void SetExecutionMode::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Command::deserialize(data, version);

    data >> uuid;
    data >> was_mode;
    data >> mode;
}
