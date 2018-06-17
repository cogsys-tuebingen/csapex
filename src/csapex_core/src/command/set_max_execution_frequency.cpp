/// HEADER
#include <csapex/command/set_max_execution_frequency.h>

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

CSAPEX_REGISTER_COMMAND_SERIALIZER(SetMaximumExecutionFrequency)

SetMaximumExecutionFrequency::SetMaximumExecutionFrequency(const AUUID& parent_uuid, const UUID &node, double frequency)
    : CommandImplementation(parent_uuid), uuid(node), frequency(frequency)
{
}

std::string SetMaximumExecutionFrequency::getDescription() const
{
    std::stringstream ss;
    ss << "set the frequency of " << uuid << " to " << frequency;
    return ss.str();
}

bool SetMaximumExecutionFrequency::doExecute()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);
    apex_assert_hard(node_handle);

    NodeStatePtr state = node_handle->getNodeState();
    was_frequency = state->getMaximumFrequency();

    state->setMaximumFrequency(frequency);

    return true;
}

bool SetMaximumExecutionFrequency::doUndo()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);
    apex_assert_hard(node_handle);

    NodeStatePtr state = node_handle->getNodeState();

    state->setMaximumFrequency(was_frequency);

    return true;
}

bool SetMaximumExecutionFrequency::doRedo()
{
    return doExecute();
}



void SetMaximumExecutionFrequency::serialize(SerializationBuffer &data, SemanticVersion& version) const
{
    Command::serialize(data, version);

    data << uuid;
    data << frequency;
}

void SetMaximumExecutionFrequency::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Command::deserialize(data, version);

    data >> uuid;
    data >> frequency;
}
