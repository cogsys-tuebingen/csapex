/// HEADER
#include <csapex/command/quit.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/model/graph_facade.h>
#include <csapex/utility/error_handling.h>
#include <csapex/utility/assert.h>
#include <csapex/command/command_serializer.h>
#include <csapex/core/csapex_core.h>

/// SYSTEM
#include <sstream>
#include <iostream>
#include <typeindex>

using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(Quit)

Quit::Quit()
{

}

bool Quit::isUndoable() const
{
    return false;
}

bool Quit::isHidden() const
{
    return true;
}

std::string Quit::getDescription() const
{
    return "Quit Request";
}

bool Quit::doExecute()
{
    core_->shutdown();
    return true;
}

bool Quit::doUndo()
{
    return true;
}

bool Quit::doRedo()
{
    return doExecute();
}

void Quit::serialize(SerializationBuffer &data) const
{
    Command::serialize(data);

}
void Quit::deserialize(const SerializationBuffer& data)
{
    Command::deserialize(data);
}
