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

bool AddBox::execute()
{
    Box::Ptr box_ = BoxManager::instance().makeBox(type_, uuid_);

    if(saved_state_) {
        box_->setState(saved_state_);
    }

    box_->hide();


    Graph::Ptr parent;
    if(parent_uuid_.empty()) {
        parent = Graph::root();
    } else {
        parent = Graph::root()->findSubGraph(parent_uuid_);
    }
    parent->addBox(box_);

    box_->init(pos_);

    return true;
}

bool AddBox::undo()
{
    Box::Ptr box_ = Graph::root()->findBox(uuid_);

    saved_state_ = box_->getState();

    Graph::Ptr parent;
    if(parent_uuid_.empty()) {
        parent = Graph::root();
    } else {
        parent = Graph::root()->findSubGraph(parent_uuid_);
    }
    parent->deleteBox(box_->UUID());

    return true;
}

bool AddBox::redo()
{
    if(execute()) {
        Box::Ptr box_ = Graph::root()->findBox(uuid_);

        box_->setState(saved_state_);
        return true;
    }

    return false;
}
