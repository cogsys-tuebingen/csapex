/// HEADER
#include <csapex/command/add_node.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/model/graph_facade.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/utility/assert.h>

using namespace csapex;
using namespace csapex::command;

AddNode::AddNode(const AUUID &parent_uuid, const std::string &type, Point pos, const UUID& uuid, NodeState::Ptr state)
    : Command(parent_uuid), type_(type), pos_(pos), uuid_(uuid)
{
    apex_assert_hard(!uuid.empty());

    if(state != nullptr) {
        NodeState::Ptr bs = std::dynamic_pointer_cast<NodeState> (state);
        saved_state_ = bs;
    }
}

std::string AddNode::getType() const
{
    return "AddNode";
}

std::string AddNode::getDescription() const
{
    return std::string("added a node of type ") + type_ + " and UUID " + uuid_.getFullName();
}


bool AddNode::doExecute()
{
    Graph* graph = getGraph();

    if(uuid_.empty()) {
        uuid_ = graph->generateUUID(type_);
    }

    NodeHandlePtr node = getNodeFactory()->makeNode(type_, uuid_, graph, saved_state_);

    if(!node) {
        return false;
    }

    NodeStatePtr state = node->getNodeState();
    state->setPos(pos_, true);

    graph->addNode(node);

    return true;
}

bool AddNode::doUndo()
{
    Graph* graph = getGraph();
    NodeHandle* node_ = graph->findNodeHandle(uuid_);

    saved_state_ = node_->getNodeStateCopy();

    graph->deleteNode(node_->getUUID());

    return true;
}

bool AddNode::doRedo()
{
    if(doExecute()) {
        getGraph()->findNodeHandle(uuid_)->setNodeState(saved_state_);
        return true;
    }

    return false;
}
