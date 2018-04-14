/// HEADER
#include <csapex/command/move_box.h>

/// COMPONENT
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_handle.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>

/// SYSTEM
#include <sstream>

using namespace csapex;
using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(MoveBox)

MoveBox::MoveBox(const AUUID& graph_uuid, const UUID& node_uuid, Point from, Point to)
    : CommandImplementation(graph_uuid), from(from), to(to), box_uuid(node_uuid)
{
}

std::string MoveBox::getDescription() const
{
    std::stringstream ss;
    ss << "moved box " << box_uuid << " from (" << from.x << ", " << from.y << ") to";
    ss << "(" << to.x << ", " << to.y << ")";
    return ss.str();
}


bool MoveBox::doExecute()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(box_uuid);
    apex_assert_hard(node_handle);

    NodeStatePtr node_state = node_handle->getNodeState();

    node_state->setPos(to);

    return true;
}

bool MoveBox::doUndo()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(box_uuid);
    apex_assert_hard(node_handle);

    NodeStatePtr node_state = node_handle->getNodeState();

    node_state->setPos(from);

    return true;
}

bool MoveBox::doRedo()
{
    return doExecute();
}

void MoveBox::serialize(SerializationBuffer &data) const
{
    Command::serialize(data);

    data << from.x << from.y;
    data << to.x << to.y;
    data << box_uuid;
}

void MoveBox::deserialize(const SerializationBuffer& data)
{
    Command::deserialize(data);

    data >> from.x >> from.y;
    data >> to.x >> to.y;
    data >> box_uuid;
}

