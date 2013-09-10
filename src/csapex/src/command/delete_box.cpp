/// HEADER
#include <csapex/command/delete_box.h>

/// COMPONENT
#include <csapex/command/delete_connection.h>
#include <csapex/model/boxed_object_constructor.h>
#include <csapex/model/box.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/graph.h>
#include <csapex/command/instanciate_subgraph_template.h>

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
    box->setState(saved_state);

    if(!box->state->template_.empty()) {
        doExecute(Command::Ptr(new command::InstanciateTemplate(box->state->template_, uuid, pos)));

    } else {
        Graph::root()->addBox(box);
    }


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
