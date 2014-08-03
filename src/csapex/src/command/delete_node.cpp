/// HEADER
#include <csapex/command/delete_node.h>

/// COMPONENT
#include <csapex/command/delete_connection.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/node_state.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/graph.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex::command;

DeleteNode::DeleteNode(const UUID& uuid)
    : Meta("delete node and connections"), uuid(uuid)
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

    locked = false;
    clear();

    BOOST_FOREACH(Input* i, node->getInputs()) {
        if(i->isConnected()) {
            add(i->removeAllConnectionsCmd());
        }
    }
    BOOST_FOREACH(Output* i, node->getOutputs()) {
        if(i->isConnected()) {
            add(i->removeAllConnectionsCmd());
        }
    }

    locked = true;

    if(Meta::doExecute()) {
        saved_state = node->getNodeStateCopy();

        graph_->deleteNode(node->getUUID());
        return true;
    }

    return false;
}

bool DeleteNode::doUndo()
{
    Node::Ptr node = BoxManager::instance().makeNode(type, uuid);

    node->setNodeState(saved_state);

    graph_->addNode(node);

    return Meta::doUndo();
}

bool DeleteNode::doRedo()
{
    if(Meta::doRedo()) {
        Node* node = graph_->findNode(uuid);
        saved_state = node->getNodeStateCopy();

        graph_->deleteNode(node->getUUID());
        return true;
    }

    return false;
}
