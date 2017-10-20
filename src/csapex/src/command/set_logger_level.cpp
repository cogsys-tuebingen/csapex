/// HEADER
#include <csapex/command/set_logger_level.h>

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

CSAPEX_REGISTER_COMMAND_SERIALIZER(SetLoggerLevel)

SetLoggerLevel::SetLoggerLevel(const AUUID& parent_uuid, const UUID &node, int level)
    : CommandImplementation(parent_uuid), uuid(node), level(level)
{
}

std::string SetLoggerLevel::getDescription() const
{
    std::stringstream ss;
    ss << "set the logger level of " << uuid << " to ";
    switch(level) {
    case 0:
        ss << "DEBUG";
        break;
    case 1:
        ss << "INFO";
        break;
    case 2:
        ss << "WARNING";
        break;
    case 3:
        ss << "ERROR";
        break;
    }

    return ss.str();
}

bool SetLoggerLevel::doExecute()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);
    apex_assert_hard(node_handle);

    NodeStatePtr state = node_handle->getNodeState();
    was_level = state->getLoggerLevel();

    state->setLoggerLevel(level);

    return true;
}

bool SetLoggerLevel::doUndo()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(uuid);
    apex_assert_hard(node_handle);

    NodeStatePtr state = node_handle->getNodeState();

    state->setLoggerLevel(was_level);

    return true;
}

bool SetLoggerLevel::doRedo()
{
    return doExecute();
}



void SetLoggerLevel::serialize(SerializationBuffer &data) const
{
    Command::serialize(data);

    data << uuid;
    data << was_level;
    data << level;
}

void SetLoggerLevel::deserialize(const SerializationBuffer& data)
{
    Command::deserialize(data);

    data >> uuid;
    data >> was_level;
    data >> level;
}
