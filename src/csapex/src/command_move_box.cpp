/// HEADER
#include <csapex/command_move_box.h>

/// COMPONENT
#include <csapex/box.h>
#include <csapex/box_manager.h>

using namespace csapex::command;

MoveBox::MoveBox(Box *box, QPoint from, QPoint to)
    : box(box), from(from), to(to)
{
    uuid = box->UUID();
}

void MoveBox::execute()
{
    box->move(to);
}

bool MoveBox::undo()
{
    box = BoxManager::instance().findBox(uuid);
    box->move(from);

    return true;
}

void MoveBox::redo()
{
    box = BoxManager::instance().findBox(uuid);
    execute();
}
