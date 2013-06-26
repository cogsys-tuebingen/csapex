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
    : PluginManager<vision_evaluator::BoxedObject>("vision_evaluator::BoxedObject"), available_elements("vision_evaluator::SelectorProxy"), dirty_(false)
{
}

void BoxManager::fill(QLayout* layout)
{
    if(!pluginsLoaded()) {
        reload();
    }

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

Box* BoxManager::makeBox(QWidget* parent, QPoint pos, const std::string& type, const std::string& uuid)
{
    std::string uuid_ = uuid;
    typedef std::pair<std::string, SelectorProxy::ProxyConstructor> Pair;
    BOOST_FOREACH(Pair p, available_elements.availableClasses()) {
        if(p.second.getType() == type) {
            if(uuid_.empty()) {
                uuid_ = makeUUID(type);
            }
            return p.second()->spawnObject(parent, pos, type, uuid);
        }
    }
    BOOST_FOREACH(SelectorProxy::Ptr p, available_elements_prototypes) {
        if(p->getType() == type) {
            if(uuid_.empty()) {
                uuid_ = makeUUID(type);
            }
            return p->spawnObject(parent, pos, type, uuid);
        }
    }

    std::cerr << "error: cannot make box, type '" << type << "' is unknown" << std::endl;
    return NULL;
}

Box* BoxManager::findBox(const std::string &uuid)
{
    BOOST_FOREACH(Box* b, container_->findChildren<Box*>()) {
        if(b->UUID() == uuid) {
            return b;
        }
    }

    return NULL;
}

Box* BoxManager::findConnectorOwner(const std::string &uuid)
{
    BOOST_FOREACH(Box* b, container_->findChildren<Box*>()) {
        if(b->getInput(uuid)) {
            return b;
        }
        if(b->getOutput(uuid)) {
            return b;
        }
    }

    return NULL;
}

void BoxManager::setContainer(QWidget *c)
{
    container_ = c;
}

QWidget* BoxManager::container()
{
    return container_;
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

std::string BoxManager::makeUUID(const std::string& name)
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
    bool change = (dirty != dirty_);

    dirty_ = dirty;

    if(change) {
        Q_EMIT dirtyChanged(dirty_);
    }
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

    assert(last->undo());

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
