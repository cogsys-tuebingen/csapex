/// HEADER
#include <csapex/command/add_fulcrum.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/fulcrum.h>
#include <csapex/model/connection.h>

/// SYSTEM
#include <sstream>

using namespace csapex;
using namespace csapex::command;

AddFulcrum::AddFulcrum(const AUUID& parent_uuid, int connection_id, int sub_section_to_split, const Point &pos, int type)
    : Command(parent_uuid), connection_id(connection_id), sub_section_to_split(sub_section_to_split), pos(pos), type(type)
{

}

std::string AddFulcrum::getType() const
{
    return "AddFulcrum";
}

std::string AddFulcrum::getDescription() const
{
    std::stringstream ss;
    ss << "added a fulcrum to connection " << connection_id;
    return ss.str();
}


bool AddFulcrum::doExecute()
{
    getGraph()->getConnectionWithId(connection_id)->addFulcrum(sub_section_to_split, pos, type);
    return true;
}

bool AddFulcrum::doUndo()
{
    getGraph()->getConnectionWithId(connection_id)->deleteFulcrum(sub_section_to_split);
    return true;
}

bool AddFulcrum::doRedo()
{
    return doExecute();
}

