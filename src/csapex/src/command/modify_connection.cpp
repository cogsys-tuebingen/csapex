/// HEADER
#include <csapex/command/modify_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/fulcrum.h>
#include <csapex/model/connection.h>

/// SYSTEM
#include <sstream>

using namespace csapex;
using namespace csapex::command;

ModifyConnection::ModifyConnection(const AUUID& parent_uuid, int connection_id, bool active)
    : Command(parent_uuid), connection_id(connection_id), active(active)
{
}

std::string ModifyConnection::getType() const
{
    return "ModifyConnection";
}

std::string ModifyConnection::getDescription() const
{
    std::stringstream ss;
    ss << "modified connection " << connection_id << " -> set active: " << active << " (was " << was_active <<  ")";
    return ss.str();
}

bool ModifyConnection::doExecute()
{
    auto c = getGraph()->getConnectionWithId(connection_id);
    was_active = c->isActive();
    c->setActive(active);
    return true;
}

bool ModifyConnection::doUndo()
{
    getGraph()->getConnectionWithId(connection_id)->setActive(was_active);
    return true;
}

bool ModifyConnection::doRedo()
{
    return doExecute();
}

