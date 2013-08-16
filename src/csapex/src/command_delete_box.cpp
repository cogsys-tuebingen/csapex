/// HEADER
#include <csapex/command_delete_box.h>

/// COMPONENT
#include <csapex/command_delete_connection.h>
#include <csapex/selector_proxy.h>
#include <csapex/box.h>
#include <csapex/box_manager.h>
#include <csapex/graph.h>

using namespace csapex::command;

DeleteBox::DeleteBox(Box *box)
    : box(box), pos(box->pos())
{
    parent = box->parentWidget();
    type = box->getType();
    uuid = box->UUID();
}

bool DeleteBox::execute()
{    
    graph = box->getGraph();

    pos = box->pos();
    remove_connections = box->removeAllConnectionsCmd();

    if(doExecute(remove_connections)) {
        saved_state = box->getState();

        graph->deleteBox(box);
        return true;
    }

    return false;
}

bool DeleteBox::undo()
{
    box = BoxManager::instance().makeBox(pos, type, uuid);
    graph->addBox(box);
    box->setState(saved_state);

    return doUndo(remove_connections);
}

bool DeleteBox::redo()
{
    refresh();

    if(doRedo(remove_connections)) {
        saved_state = box->getState();

        graph->deleteBox(box);
        return true;
    }

    return false;
}

void DeleteBox::refresh()
{
    box = graph->findBox(uuid);
}
