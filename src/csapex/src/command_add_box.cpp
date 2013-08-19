/// HEADER
#include <csapex/command_add_box.h>

/// COMPONENT
#include <csapex/command.h>
#include <csapex/selector_proxy.h>
#include <csapex/box.h>
#include <csapex/box_manager.h>
#include <csapex/graph.h>

using namespace csapex::command;

AddBox::AddBox(SelectorProxy::Ptr selector, QPoint pos, Memento::Ptr state, const std::string &parent_uuid, const std::string& uuid)
    : selector_(selector), pos_(pos), parent_uuid_(parent_uuid)
{
    if(state != Memento::NullPtr) {
        Box::State::Ptr bs = boost::dynamic_pointer_cast<Box::State> (state);
        if(!uuid.empty()) {
            bs->uuid_ = uuid;
        }
        saved_state_ = bs;
    }

    uuid_ = uuid.empty() ? BoxManager::instance().makeUUID(selector->getType()) : uuid;
}

bool AddBox::execute()
{
    box_ = BoxManager::instance().makeBox(pos_, selector_->getType(), uuid_);

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

    return true;
}

bool AddBox::undo()
{
    refresh();

    saved_state_ = box_->getState();

    Graph::Ptr parent;
    if(parent_uuid_.empty()) {
        parent = Graph::root();
    } else {
        parent = Graph::root()->findSubGraph(parent_uuid_);
    }
    parent->deleteBox(box_);

    return true;
}

bool AddBox::redo()
{
    if(execute()) {
        box_->setState(saved_state_);
        return true;
    }

    return false;
}

void AddBox::refresh()
{
    box_ = Graph::root()->findBox(uuid_);
}
