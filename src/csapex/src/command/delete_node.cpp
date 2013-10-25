/// HEADER
#include <csapex/command/delete_node.h>

/// COMPONENT
#include <csapex/command/delete_connection.h>
#include <csapex/model/boxed_object_constructor.h>
#include <csapex/model/box.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/graph.h>

using namespace csapex::command;

DeleteNode::DeleteNode(const std::string &uuid)
    : uuid(uuid)
{
}

bool DeleteNode::doExecute()
{
    Box* box = graph_->findNode(uuid)->getBox();

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

bool DeleteNode::doUndo()
{
    Box::Ptr box = BoxManager::instance().makeBox(type, uuid);
    box->setState(saved_state);

    graph_->addBox(box);

    box->init(pos);

    return Command::undoCommand(graph_, remove_connections);
}

bool DeleteNode::doRedo()
{
    if(Command::redoCommand(graph_, remove_connections)) {
        Box* box = graph_->findNode(uuid)->getBox();
        saved_state = box->getState();

        graph_->deleteBox(box->UUID());
        return true;
    }

    return false;
}
