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

bool MoveBox::execute()
{
    box->move(to);

    return true;
}

bool MoveBox::undo()
{
    box = BoxManager::instance().findBox(uuid);
    box->move(from);

    return true;
}

bool MoveBox::redo()
{
    box = BoxManager::instance().findBox(uuid);
    return execute();
}
