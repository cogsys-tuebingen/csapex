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
    graph = box->getGraph();
}

bool MoveBox::execute()
{
    box->move(to);

    return true;
}

bool MoveBox::undo()
{
    box = graph->findBox(uuid);
    box->move(from);

    return true;
}

bool MoveBox::redo()
{
    box = graph->findBox(uuid);
    return execute();
}
