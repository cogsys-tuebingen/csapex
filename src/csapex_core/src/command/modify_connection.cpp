/// HEADER
#include <csapex/command/modify_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/fulcrum.h>
#include <csapex/model/connection.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/io/std_io.h>

/// SYSTEM
#include <sstream>

using namespace csapex;
using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(ModifyConnection)

ModifyConnection::ModifyConnection(const AUUID& parent_uuid, int connection_id, bool active)
    : CommandImplementation(parent_uuid), connection_id(connection_id), active(active)
{
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



void ModifyConnection::serialize(SerializationBuffer &data) const
{
    Command::serialize(data);

    data << connection_id;
    data << active;
    data << was_active;
}

void ModifyConnection::deserialize(const SerializationBuffer& data)
{
    Command::deserialize(data);

    data >> connection_id;
    data >> active;
    data >> was_active;
}
