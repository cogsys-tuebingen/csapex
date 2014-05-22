/// HEADER
#include <csapex/command/move_box.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/view/box.h>
#include <csapex/model/graph.h>
#include <csapex/view/widget_controller.h>

using namespace csapex::command;

MoveBox::MoveBox(NodeBox* box, QPoint to)
    : from(box->key_point), to(to), uuid(box->getNode()->getUUID())
{
}

std::string MoveBox::getType() const
{
    return "MoveBox";
}

std::string MoveBox::getDescription() const
{
    std::stringstream ss;
    ss << "moved box " << uuid << " from (" << from.x() << ", " << from.y() << ") to";
    ss << "(" << to.x() << ", " << to.y() << ")";
    return ss.str();
}


bool MoveBox::doExecute()
{
    NodeBox* box = widget_ctrl_->getBox(uuid);
    box->clearFocus();
    box->move(to);
    box->key_point = to;

    return true;
}

bool MoveBox::doUndo()
{
    NodeBox* box = widget_ctrl_->getBox(uuid);
    box->clearFocus();
    box->move(from);
    box->key_point = from;

    return true;
}

bool MoveBox::doRedo()
{
    return doExecute();
}
