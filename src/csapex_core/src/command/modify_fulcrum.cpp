/// HEADER
#include <csapex/command/modify_fulcrum.h>

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

CSAPEX_REGISTER_COMMAND_SERIALIZER(ModifyFulcrum)

ModifyFulcrum::ModifyFulcrum(const AUUID& parent_uuid, int connection_id, int fulcrum_id,
                             int f_type, const Point &f_handle_in, const Point &f_handle_out,
                             int t_type, const Point &t_handle_in, const Point &t_handle_out)
    : CommandImplementation(parent_uuid), connection_id(connection_id), fulcrum_id(fulcrum_id),
      f_type(f_type), f_in(f_handle_in), f_out(f_handle_out),
      t_type(t_type), t_in(t_handle_in), t_out(t_handle_out)
{
}

std::string ModifyFulcrum::getDescription() const
{
    std::stringstream ss;
    ss << "modified fulcrum " << fulcrum_id << " of connection " << connection_id;
    ss << "(type=" << t_type << ", in: " << t_in.x << "/" << t_in.y << ", out: " << t_out.x << "/" << t_out.y << ")";
    return ss.str();
}

bool ModifyFulcrum::doExecute()
{
    getGraph()->getConnectionWithId(connection_id)->modifyFulcrum(fulcrum_id, t_type, t_in, t_out);
    return true;
}

bool ModifyFulcrum::doUndo()
{
    getGraph()->getConnectionWithId(connection_id)->modifyFulcrum(fulcrum_id, f_type, f_in, f_out);
    return true;
}

bool ModifyFulcrum::doRedo()
{
    return doExecute();
}



void ModifyFulcrum::serialize(SerializationBuffer &data, SemanticVersion& version) const
{
    Command::serialize(data, version);

    data << connection_id;
    data <<fulcrum_id;

    data << f_type;
    data << f_in.x << f_in.y;
    data << f_out.x << f_out.y;

    data << t_type;
    data << t_in.x << t_in.y;
    data << t_out.x << t_out.y;
}

void ModifyFulcrum::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Command::deserialize(data, version);

    data >> connection_id;
    data >>fulcrum_id;

    data >> f_type;
    data >> f_in.x >> f_in.y;
    data >> f_out.x >> f_out.y;

    data >> t_type;
    data >> t_in.x >> t_in.y;
    data >> t_out.x >> t_out.y;
}
