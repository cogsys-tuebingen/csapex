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

bool DeleteBox::execute(Graph& graph)
{    
    remove_connections = box->removeAllConnectionsCmd();

    if(doExecute(graph, remove_connections)) {
        saved_state = box->getState();

        graph.deleteBox(box);
        return true;
    }

    return false;
}

bool DeleteBox::undo(Graph& graph)
{
    box = BoxManager::instance().makeBox(pos, type, uuid);
    graph.addBox(box);
    box->setState(saved_state);

    return doUndo(graph, remove_connections);
}

bool DeleteBox::redo(Graph& graph)
{
    refresh(graph);

    if(doRedo(graph, remove_connections)) {
        saved_state = box->getState();

        graph.deleteBox(box);
        return true;
    }

    return false;
}

void DeleteBox::refresh(Graph& graph)
{
    box = graph.findBox(uuid);
}
