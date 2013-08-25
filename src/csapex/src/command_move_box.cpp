/// HEADER
#include <csapex/command_move_box.h>

/// COMPONENT
#include <csapex/box.h>
#include <csapex/graph.h>

using namespace csapex::command;

MoveBox::MoveBox(Box* box, QPoint from, QPoint to)
    : from(from), to(to)
{
    uuid = box->UUID();
}

bool MoveBox::execute()
{
    Box::Ptr box = Graph::root()->findBox(uuid);
    box->move(to);

    return true;
}

bool MoveBox::undo()
{
    Box::Ptr box = Graph::root()->findBox(uuid);
    box->move(from);

    return true;
}

bool MoveBox::redo()
{
    return execute();
}
