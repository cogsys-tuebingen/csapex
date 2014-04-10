/// HEADER
#include <csapex/command/delete_node.h>

/// COMPONENT
#include <csapex/command/delete_connection.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/node_state.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/graph.h>

using namespace csapex::command;

DeleteNode::DeleteNode(const UUID& uuid)
    : uuid(uuid)
{
}

std::string DeleteNode::getType() const
{
    return "DeleteNode";
}

std::string DeleteNode::getDescription() const
{
    return std::string("deleted node ") + uuid.getFullName();
}


bool DeleteNode::doExecute()
{
    Node* node = graph_->findNode(uuid);

    type = node->getType();

    remove_connections = node->removeAllConnectionsCmd();

    if(Command::executeCommand(graph_, widget_ctrl_, remove_connections)) {
        saved_state = node->getNodeState();

        graph_->deleteNode(node->getUUID());
        return true;
    }

    return false;
}

bool DeleteNode::doUndo()
{
    Node::Ptr node = BoxManager::instance().makeNode(type, uuid);

    node->setNodeStateLater(saved_state);

    graph_->addNode(node);

    return Command::undoCommand(graph_, widget_ctrl_, remove_connections);
}

bool DeleteNode::doRedo()
{
    if(Command::redoCommand(graph_, widget_ctrl_, remove_connections)) {
        Node* node = graph_->findNode(uuid);
        saved_state = node->getNodeState();

        graph_->deleteNode(node->getUUID());
        return true;
    }

    return false;
}
