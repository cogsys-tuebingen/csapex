/// HEADER
#include <csapex/command_delete_box.h>

/// COMPONENT
#include <csapex/command_delete_connection.h>
#include <csapex/selector_proxy.h>
#include <csapex/box.h>
#include <csapex/box_manager.h>
#include <csapex/graph.h>

using namespace csapex::command;

DeleteBox::DeleteBox(const std::string &uuid)
    : uuid(uuid)
{
    Box::Ptr box = Graph::root()->findBox(uuid);

    parent = box->parentWidget();
    type = box->getType();
}

bool DeleteBox::execute()
{
    Box::Ptr box = Graph::root()->findBox(uuid);

    pos = box->pos();
    remove_connections = box->removeAllConnectionsCmd();

    if(doExecute(remove_connections)) {
        saved_state = box->getState();

        Graph::root()->deleteBox(box->UUID());
        return true;
    }

    return false;
}

bool DeleteBox::undo()
{
    Box::Ptr box = BoxManager::instance().makeBox(pos, type, uuid);
    Graph::root()->addBox(box);
    box->setState(saved_state);

    return doUndo(remove_connections);
}

bool DeleteBox::redo()
{
    if(doRedo(remove_connections)) {
        Box::Ptr box = Graph::root()->findBox(uuid);
        saved_state = box->getState();

        Graph::root()->deleteBox(box->UUID());
        return true;
    }

    return false;
}
