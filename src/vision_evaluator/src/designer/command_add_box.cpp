/// HEADER
#include "command_add_box.h"

/// COMPONENT
#include "command.h"
#include "selector_proxy.h"
#include "box.h"
#include "box_manager.h"

using namespace vision_evaluator::command;

AddBox::AddBox(SelectorProxy* selector, QWidget* parent, QPoint pos)
    : selector(selector), parent(parent), pos(pos)
{
    uuid = BoxManager::instance().makeUUID(selector->getType());
    type = selector->getType();
}

void AddBox::execute()
{
    box = selector->spawnObject(parent, pos, type, uuid);
    BoxManager::instance().setDirty(true);
}

bool AddBox::undo()
{
    refresh();

    saved_state = box->getState();
    box->stop();
    delete box;
    box = NULL;

    return true;
}

void AddBox::redo()
{
    execute();
    box->setState(saved_state);
}

void AddBox::refresh()
{
    box = BoxManager::instance().findBox(uuid);
}
