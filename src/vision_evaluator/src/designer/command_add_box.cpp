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
    uuid = BoxManager::instance().makeUUID(selector->name());
}

void AddBox::execute() {
    box = selector->spawnObject(parent, pos, uuid);
}

void AddBox::undo() {
    saved_state = box->saveState();
    box->stop();
    delete box;
}

void AddBox::redo() {
    box = selector->spawnObject(parent, pos, uuid);
    box->loadState(saved_state);
}
