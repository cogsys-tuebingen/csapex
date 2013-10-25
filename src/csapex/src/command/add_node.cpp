/// HEADER
#include <csapex/command/add_node.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/boxed_object_constructor.h>
#include <csapex/model/box.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/graph.h>

using namespace csapex::command;

AddNode::AddNode(const std::string &type, QPoint pos, const std::string &parent_uuid, const std::string& uuid, Memento::Ptr state)
    : type_(type), pos_(pos), parent_uuid_(parent_uuid), uuid_(uuid)
{
    assert(!uuid.empty());

    if(state != Memento::NullPtr) {
        Box::State::Ptr bs = boost::dynamic_pointer_cast<Box::State> (state);
        saved_state_ = bs;
    }
}

bool AddNode::doExecute()
{
    if(uuid_.empty()) {
        uuid_ = graph_->makeUUID(type_);
    }

    Box::Ptr box_ = BoxManager::instance().makeBox(type_, uuid_);
    assert(box_->getType() == type_);

    if(saved_state_) {
        box_->setState(saved_state_);
    }

    box_->hide();


    if(parent_uuid_.empty()) {
        graph_->addBox(box_);
    } else {
        graph_->findSubGraph(parent_uuid_)->addBox(box_);
    }

    box_->init(pos_);

    return true;
}

bool AddNode::doUndo()
{
    Box* box_ = graph_->findNode(uuid_)->getBox();

    saved_state_ = box_->getState();


    if(parent_uuid_.empty()) {
        graph_->deleteBox(box_->UUID());
    } else {
        graph_->findSubGraph(parent_uuid_)->deleteBox(box_->UUID());
    }

    return true;
}

bool AddNode::doRedo()
{
    if(doExecute()) {
        Box* box_ = graph_->findNode(uuid_)->getBox();

        box_->setState(saved_state_);
        return true;
    }

    return false;
}
