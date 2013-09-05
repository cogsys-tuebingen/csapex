/// HEADER
#include <csapex/command_delete_fulcrum.h>

/// COMPONENT
#include <csapex/command.h>
#include <csapex/graph.h>

using namespace csapex::command;

DeleteFulcrum::DeleteFulcrum(int connection_id, int fulcrum_id)
    : connection_id(connection_id), fulcrum_id(fulcrum_id)
{
    pos = Graph::root()->getConnectionWithId(connection_id)->getFulcrum(fulcrum_id);
}

bool DeleteFulcrum::execute()
{
    Graph::root()->getConnectionWithId(connection_id)->deleteFulcrum(fulcrum_id);
    return true;
}

bool DeleteFulcrum::undo()
{
    Graph::root()->getConnectionWithId(connection_id)->addFulcrum(fulcrum_id, pos);
    return true;
}

bool DeleteFulcrum::redo()
{
    return execute();
}


