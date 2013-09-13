/// HEADER
#include <csapex/command/add_box.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/boxed_object_constructor.h>
#include <csapex/model/box.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/graph.h>

using namespace csapex::command;

AddBox::AddBox(const std::string &type, QPoint pos, const std::string &parent_uuid, const std::string& uuid, Memento::Ptr state)
    : type_(type), pos_(pos), parent_uuid_(parent_uuid), uuid_(uuid)
{
    assert(!uuid.empty());

    if(state != Memento::NullPtr) {
        Box::State::Ptr bs = boost::dynamic_pointer_cast<Box::State> (state);
        if(!uuid.empty()) {
            bs->uuid_ = uuid;
        }
        saved_state_ = bs;
    }
}

bool AddBox::doExecute()
{
    if(uuid_.empty()) {
        uuid_ = graph_->makeUUID(type_);
    }

    Box::Ptr box_ = BoxManager::instance().makeBox(type_, uuid_);

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

bool AddBox::doUndo()
{
    Box::Ptr box_ = graph_->findBox(uuid_);

    saved_state_ = box_->getState();


    if(parent_uuid_.empty()) {
        graph_->deleteBox(box_->UUID());
    } else {
        graph_->findSubGraph(parent_uuid_)->deleteBox(box_->UUID());
    }

    return true;
}

bool AddBox::doRedo()
{
    if(doExecute()) {
        Box::Ptr box_ = graph_->findBox(uuid_);

        box_->setState(saved_state_);
        return true;
    }

    return false;
}
