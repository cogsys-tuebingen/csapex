/// HEADER
#include <csapex/command_add_box.h>

/// COMPONENT
#include <csapex/command.h>
#include <csapex/selector_proxy.h>
#include <csapex/box.h>
#include <csapex/box_manager.h>
#include <csapex/graph.h>

using namespace csapex::command;

AddBox::AddBox(Graph& graph, SelectorProxy::Ptr selector, QPoint pos, Memento::Ptr state, const std::string& _uuid)
    : graph_(graph), selector(selector), pos(pos)
{
    if(state != Memento::NullPtr) {
        Box::State::Ptr bs = boost::dynamic_pointer_cast<Box::State> (state);
        if(!_uuid.empty()) {
            bs->uuid_ = _uuid;
        }
        saved_state = bs;
    }

    uuid = _uuid.empty() ? BoxManager::instance().makeUUID(selector->getType()) : _uuid;
    type = selector->getType();
}

bool AddBox::execute()
{
    box = BoxManager::instance().makeBox(pos, type, uuid);

    if(saved_state) {
        box->setState(saved_state);
    }

    box->hide();

    graph_.addBox(box);

    return true;
}

bool AddBox::undo()
{
    refresh();

    saved_state = box->getState();
    graph_.deleteBox(box);

    return true;
}

bool AddBox::redo()
{
    if(execute()) {
        box->setState(saved_state);
        return true;
    }

    return false;
}

void AddBox::refresh()
{
    box = graph_.findBox(uuid);
}
