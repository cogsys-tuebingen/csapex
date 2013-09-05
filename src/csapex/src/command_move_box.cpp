/// HEADER
#include <csapex/command_move_box.h>

/// COMPONENT
#include <csapex/box.h>
#include <csapex/graph.h>

using namespace csapex::command;

MoveBox::MoveBox(Box* box, QPoint to)
    : from(box->key_point), to(to)
{
    uuid = box->UUID();
}

bool MoveBox::execute()
{
    Box::Ptr box = Graph::root()->findBox(uuid);
    box->clearFocus();
    box->move(to);
    box->key_point = to;

    return true;
}

bool MoveBox::undo()
{
    Box::Ptr box = Graph::root()->findBox(uuid);
    box->clearFocus();
    box->move(from);
    box->key_point = from;

    return true;
}

bool MoveBox::redo()
{
    return execute();
}
