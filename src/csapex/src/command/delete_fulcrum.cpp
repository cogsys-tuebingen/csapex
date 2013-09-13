/// HEADER
#include <csapex/command/delete_fulcrum.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>

using namespace csapex::command;

DeleteFulcrum::DeleteFulcrum(int connection_id, int fulcrum_id)
    : connection_id(connection_id), fulcrum_id(fulcrum_id)
{
}

bool DeleteFulcrum::doExecute()
{
    pos = graph_->getConnectionWithId(connection_id)->getFulcrum(fulcrum_id);
    graph_->getConnectionWithId(connection_id)->deleteFulcrum(fulcrum_id);
    return true;
}

bool DeleteFulcrum::doUndo()
{
    graph_->getConnectionWithId(connection_id)->addFulcrum(fulcrum_id, pos);
    return true;
}

bool DeleteFulcrum::doRedo()
{
    return doExecute();
}


