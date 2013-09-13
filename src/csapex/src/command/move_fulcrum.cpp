/// HEADER
#include <csapex/command/move_fulcrum.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>

using namespace csapex::command;

MoveFulcrum::MoveFulcrum(int connection_id, int fulcrum_id, const QPoint &from, const QPoint &to)
    : connection_id(connection_id), fulcrum_id(fulcrum_id), from(from), to(to)
{
}

bool MoveFulcrum::doExecute()
{
    graph_->getConnectionWithId(connection_id)->moveFulcrum(fulcrum_id, to);
    return true;
}

bool MoveFulcrum::doUndo()
{
    graph_->getConnectionWithId(connection_id)->moveFulcrum(fulcrum_id, from);
    return true;
}

bool MoveFulcrum::doRedo()
{
    return doExecute();
}
