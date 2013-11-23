/// HEADER
#include <csapex/command/add_node.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node_state.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>

using namespace csapex::command;

AddNode::AddNode(const std::string &type, QPoint pos, const std::string &parent_uuid, const std::string& uuid, NodeState::Ptr state)
    : type_(type), pos_(pos), parent_uuid_(parent_uuid), uuid_(uuid)
{
    assert(!uuid.empty());

    if(state != MementoNullPtr) {
        NodeState::Ptr bs = boost::dynamic_pointer_cast<NodeState> (state);
        saved_state_ = bs;
    }
}

std::string AddNode::getType() const
{
    return "AddNode";
}

std::string AddNode::getDescription() const
{
    return std::string("added a node of type ") + type_ + " and UUID " + uuid_;
}


bool AddNode::doExecute()
{
    if(uuid_.empty()) {
        uuid_ = graph_->makeUUID(type_);
    }

    Node::Ptr node = BoxManager::instance().makeNode(type_, uuid_);

    assert(node->getType() == type_);

    if(saved_state_) {
        node->setNodeStateLater(saved_state_);
    }

    node->setPosition(pos_);

//    if(parent_uuid_.empty()) {
        graph_->addNode(node);
//    } else {
//        graph_->findSubGraph(parent_uuid_)->addBox(box);
//    }

    return true;
}

bool AddNode::doUndo()
{
    Node* node_ = graph_->findNode(uuid_);

    saved_state_ = node_->getNodeState();


    if(parent_uuid_.empty()) {
        graph_->deleteNode(node_->UUID());
    } else {
//        graph_->findSubGraph(parent_uuid_)->deleteBox(box_->UUID());
    }

    return true;
}

bool AddNode::doRedo()
{
    if(doExecute()) {
        graph_->findNode(uuid_)->setNodeStateLater(saved_state_);
        return true;
    }

    return false;
}
