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
}

bool DeleteBox::doExecute()
{
    Box::Ptr box = graph_->findBox(uuid);

    parent = box->parentWidget();
    type = box->getType();

    pos = box->pos();
    remove_connections = box->removeAllConnectionsCmd();

    if(Command::executeCommand(graph_, remove_connections)) {
        saved_state = box->getState();

        graph_->deleteBox(box->UUID());
        return true;
    }

    return false;
}

bool DeleteBox::doUndo()
{
    Box::Ptr box = BoxManager::instance().makeBox(type, uuid);
    box->setState(saved_state);

    graph_->addBox(box);

    box->init(pos);

    return Command::undoCommand(graph_, remove_connections);
}

bool DeleteBox::doRedo()
{
    if(Command::redoCommand(graph_, remove_connections)) {
        Box::Ptr box = graph_->findBox(uuid);
        saved_state = box->getState();

        graph_->deleteBox(box->UUID());
        return true;
    }

    return false;
}
