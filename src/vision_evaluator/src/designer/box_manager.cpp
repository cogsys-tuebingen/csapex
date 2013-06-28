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
    : PluginManager<vision_evaluator::BoxedObject>("vision_evaluator::BoxedObject"), dirty_(false)
{
}

bool BoxManager::pluginsLoaded()
{
    bool r = PluginManager<BoxedObject>::pluginsLoaded();

    return r && unloaded_elements.empty();
}

void BoxManager::reload()
{
    PluginManager<BoxedObject>::reload();

    foreach(SelectorProxy::ProxyConstructor c, unloaded_elements) {
        register_box_type(SelectorProxy::Ptr(c()));
    }

    unloaded_elements.clear();
}

void BoxManager::fill(QLayout* layout)
{
    if(!pluginsLoaded()) {
        reload();
    }

    BOOST_FOREACH(SelectorProxy::Ptr p, available_elements_prototypes) {
        layout->addWidget(p->clone());
    }
}

void BoxManager::fill(QMenu* menu)
{
    if(!pluginsLoaded()) {
        reload();
    }

    std::map<std::string, std::vector<SelectorProxy::Ptr> > map;
    std::set<std::string> categories;

    std::string no_cat = "General";

    categories.insert(no_cat);

    foreach(SelectorProxy::Ptr p, available_elements_prototypes) {
        std::string cat = p->getCategory();
        if(cat.empty()) {
            cat = no_cat;
        }
        map[cat].push_back(p);
        categories.insert(cat);
    }

    foreach(const std::string& cat, categories) {
        QMenu* submenu = new QMenu(cat.c_str());
        menu->addMenu(submenu);

        foreach(const SelectorProxy::Ptr& proxy, map[cat]) {
            submenu->addAction(proxy->getType().c_str());
        }
    }

}

void BoxManager::register_box_type(SelectorProxy::ProxyConstructor provider)
{
    unloaded_elements.push_back(provider);
}

void BoxManager::register_box_type(SelectorProxy::Ptr provider)
{
    available_elements_prototypes.push_back(provider);
}

void BoxManager::startPlacingBox(const std::string &type)
{
    foreach(SelectorProxy::Ptr p, available_elements_prototypes) {
        if(p->getType() == type) {
            p->startObjectPositioning();
            return;
        }
    }
}

Box* BoxManager::makeBox(QWidget* parent, QPoint pos, const std::string& type, const std::string& uuid)
{
    std::string uuid_ = uuid;
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
