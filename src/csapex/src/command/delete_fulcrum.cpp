/// HEADER
#include <csapex/command/delete_fulcrum.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/connection.h>
#include <csapex/model/fulcrum.h>

/// SYSTEM
#include <sstream>

using namespace csapex;
using namespace csapex::command;

DeleteFulcrum::DeleteFulcrum(const AUUID& parent_uuid, int connection_id, int fulcrum_id)
    : Command(parent_uuid), connection_id(connection_id), fulcrum_id(fulcrum_id)
{
}

std::string DeleteFulcrum::getType() const
{
    return "DeleteFulcrum";
}

std::string DeleteFulcrum::getDescription() const
{
    std::stringstream ss;
    ss << "deleted fulcrum " << fulcrum_id << "  from connection " << connection_id;
    return ss.str();
}

bool DeleteFulcrum::doExecute()
{
    Fulcrum::Ptr f = getGraph()->getConnectionWithId(connection_id)->getFulcrum(fulcrum_id);
    pos = f->pos();
    type = f->type();
    in = f->handleIn();
    out = f->handleOut();
    getGraph()->getConnectionWithId(connection_id)->deleteFulcrum(fulcrum_id);
    return true;
}

bool DeleteFulcrum::doUndo()
{
    getGraph()->getConnectionWithId(connection_id)->addFulcrum(fulcrum_id, pos, type, in, out);
    return true;
}

bool DeleteFulcrum::doRedo()
{
    return doExecute();
}


