/// HEADER
#include <csapex/command/move_box.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_handle.h>

/// SYSTEM
#include <sstream>

using namespace csapex;
using namespace csapex::command;

MoveBox::MoveBox(const AUUID& graph_uuid, const UUID& node_uuid, Point from, Point to)
    : Command(graph_uuid), from(from), to(to), box_uuid(node_uuid)
{
}

std::string MoveBox::getType() const
{
    return "MoveBox";
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
