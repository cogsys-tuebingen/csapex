/// HEADER
#include <csapex/command_add_box.h>

/// COMPONENT
#include <csapex/command.h>
#include <csapex/selector_proxy.h>
#include <csapex/box.h>
#include <csapex/box_manager.h>
#include <csapex/graph.h>

using namespace csapex::command;

AddBox::AddBox(SelectorProxy* selector, QWidget* parent, QPoint pos)
    : selector(selector), parent(parent), pos(pos)
{
    uuid = BoxManager::instance().makeUUID(selector->getType());
    type = selector->getType();
}

bool AddBox::execute(Graph& graph)
{
    box = BoxManager::instance().makeBox(pos, type, uuid);
    graph.addBox(box);

    return true;
}

bool AddBox::undo(Graph& graph)
{
    refresh(graph);

    saved_state = box->getState();
    graph.deleteBox(box);

    return true;
}

bool AddBox::redo(Graph& graph)
{
    if(execute(graph)) {
        box->setState(saved_state);
        return true;
    }

    return false;
}

void AddBox::refresh(Graph& graph)
{
    box = graph.findBox(uuid);
}
