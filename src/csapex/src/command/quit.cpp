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

/// SYSTEM
#include <sstream>
#include <iostream>
#include <typeindex>

using namespace csapex::command;

REGISTER_COMMAND_SERIALIZER(Quit)

Quit::Quit()
{

}

bool Quit::isUndoable() const
{
    return false;
}

std::string Quit::getDescription() const
{
    return "Quit Request";
}

bool Quit::doExecute()
{
    csapex::error_handling::stop_request()();
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
