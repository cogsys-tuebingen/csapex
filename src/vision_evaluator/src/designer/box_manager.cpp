/// HEADER
#include "box_manager.h"

/// COMPONENT
#include "selector_proxy.h"
#include "box.h"

/// SYSTEM
#include <boost/foreach.hpp>
#include <stack>

using namespace vision_evaluator;

BoxManager::BoxManager()
    : available_elements("vision_evaluator::SelectorProxy"), dirty_(false)
{
}

void BoxManager::fill(QLayout* layout)
{
    typedef std::pair<std::string, SelectorProxy::ProxyConstructor> Pair;
    BOOST_FOREACH(Pair p, available_elements.availableClasses()) {
        layout->addWidget(p.second());
    }
    BOOST_FOREACH(SelectorProxy::Ptr p, available_elements_prototypes) {
        layout->addWidget(p->clone());
    }
}

void BoxManager::register_box_type(SelectorProxy::ProxyConstructor provider)
{
    available_elements.registerConstructor(provider);
}

void BoxManager::register_box_type(SelectorProxy::Ptr provider)
{
    available_elements_prototypes.push_back(provider);
}

void BoxManager::execute(Command::Ptr command)
{
    command->execute();
    done.push(command);

    while(!undone.empty()) {
        undone.pop();
    }

    Q_EMIT stateChanged();
}

std::string BoxManager::makeUUID(const std::string &name)
{
    int& last_id = uuids[name];
    ++last_id;

    std::stringstream ss;
    ss << name << "_" << last_id;

    return ss.str();
}

bool BoxManager::isDirty()
{
    return dirty_;
}

void BoxManager::setDirty(bool dirty)
{
    dirty_ = dirty;
}

bool BoxManager::canUndo()
{
    return !done.empty();
}

bool BoxManager::canRedo()
{
    return !undone.empty();
}

void BoxManager::undo()
{
    if(!canUndo()) {
        return;
    }

    Command::Ptr last = done.top();
    done.pop();

    last->undo();

    undone.push(last);

    Q_EMIT stateChanged();
}

void BoxManager::redo()
{
    if(!canRedo()) {
        return;
    }

    Command::Ptr last = undone.top();
    undone.pop();

    last->redo();

    done.push(last);

    Q_EMIT stateChanged();
}
