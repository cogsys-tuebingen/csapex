/// HEADER
#include <csapex/command/set_logger_level.h>

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

SetLoggerLevel::SetLoggerLevel(const AUUID& parent_uuid, const UUID &node, int level)
    : Command(parent_uuid), uuid(node), level(level)
{
}

std::string SetLoggerLevel::getType() const
{
    return "SetLoggerLevel";
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

