/// HEADER
#include <csapex/command_add_box.h>

/// COMPONENT
#include <csapex/command.h>
#include <csapex/selector_proxy.h>
#include <csapex/box.h>
#include <csapex/box_manager.h>

using namespace csapex::command;

AddBox::AddBox(SelectorProxy* selector, QWidget* parent, QPoint pos)
    : selector(selector), parent(parent), pos(pos)
{
    uuid = BoxManager::instance().makeUUID(selector->getType());
    type = selector->getType();
}

bool AddBox::execute()
{
    box = selector->spawnObject(parent, pos, type, uuid);

    return true;
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

bool AddBox::redo()
{
    if(execute()) {
        box->setState(saved_state);
        return true;
    }

    return false;
}

void AddBox::refresh()
{
    box = BoxManager::instance().findBox(uuid);
}
