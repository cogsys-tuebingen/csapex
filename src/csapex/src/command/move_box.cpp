/// HEADER
#include <csapex/command/move_box.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/view/box.h>
#include <csapex/model/graph.h>

using namespace csapex::command;

MoveBox::MoveBox(Box* box, QPoint to)
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
    Box* box = graph_->findNode(uuid)->getBox();
    box->clearFocus();
    box->move(to);
    box->key_point = to;

    return true;
}

bool MoveBox::doUndo()
{
    Box* box = graph_->findNode(uuid)->getBox();
    box->clearFocus();
    box->move(from);
    box->key_point = from;

    return true;
}

bool MoveBox::doRedo()
{
    return doExecute();
}
