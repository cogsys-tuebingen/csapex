/// HEADER
#include <csapex/command/move_fulcrum.h>

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

CSAPEX_REGISTER_COMMAND_SERIALIZER(MoveFulcrum)

MoveFulcrum::MoveFulcrum(const AUUID& parent_uuid, int connection_id, int fulcrum_id, const Point &from, const Point &to)
    : CommandImplementation(parent_uuid), connection_id(connection_id), fulcrum_id(fulcrum_id), from(from), to(to)
{
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


void MoveFulcrum::serialize(SerializationBuffer &data) const
{
    Command::serialize(data);

    data << connection_id;
    data << fulcrum_id;
    data << from.x << from.y;
    data << to.x << to.y;
}

void MoveFulcrum::deserialize(const SerializationBuffer& data)
{
    Command::deserialize(data);

    data >> connection_id;
    data >> fulcrum_id;
    data >> from.x >> from.y;
    data >> to.x >> to.y;
}
