/// HEADER
#include <csapex/command/move_fulcrum.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/fulcrum.h>
#include <csapex/model/connection.h>

/// SYSTEM
#include <sstream>

using namespace csapex;
using namespace csapex::command;

MoveFulcrum::MoveFulcrum(const AUUID& parent_uuid, int connection_id, int fulcrum_id, const Point &from, const Point &to)
    : Command(parent_uuid), connection_id(connection_id), fulcrum_id(fulcrum_id), from(from), to(to)
{
}

std::string MoveFulcrum::getType() const
{
    return "MoveFulcrum";
}

std::string MoveFulcrum::getDescription() const
{
    std::stringstream ss;
    ss << "moved fulcrum " << fulcrum_id << " of connection " << connection_id;
    return ss.str();
}

bool MoveFulcrum::doExecute()
{
    return true;
}

bool MoveFulcrum::doUndo()
{
    getGraph()->getConnectionWithId(connection_id)->moveFulcrum(fulcrum_id, from, false);
    return true;
}

bool MoveFulcrum::doRedo()
{
    getGraph()->getConnectionWithId(connection_id)->moveFulcrum(fulcrum_id, to, false);
    return doExecute();
}
