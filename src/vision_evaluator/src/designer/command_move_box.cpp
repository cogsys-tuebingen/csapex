/// HEADER
#include "command_move_box.h"

/// COMPONENT
#include "selector_proxy.h"
#include "box.h"
#include "box_manager.h"

using namespace vision_evaluator::command;

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
