/// HEADER
#include <csapex/command_move_box.h>

/// COMPONENT
#include <csapex/box.h>
#include <csapex/graph.h>

using namespace csapex::command;

MoveBox::MoveBox(Box *box, QPoint from, QPoint to)
    : box(box), from(from), to(to)
{
    uuid = box->UUID();
}

bool MoveBox::execute(Graph& graph)
{
    box->move(to);

    return true;
}

bool MoveBox::undo(Graph& graph)
{
    box = graph.findBox(uuid);
    box->move(from);

    return true;
}

bool MoveBox::redo(Graph& graph)
{
    box = graph.findBox(uuid);
    return execute(graph);
}
