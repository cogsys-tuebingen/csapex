/// HEADER
#include <csapex/box_manager.h>

/// COMPONENT
#include <csapex/selector_proxy.h>
#include <csapex/command_meta.h>
#include <csapex/command_delete_box.h>
#include <csapex/box.h>
#include <csapex/box_group.h>
#include <csapex/graph.h>
#include <csapex/sub_graph.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QApplication>
#include <QTreeWidget>
#include <stack>

using namespace csapex;

BoxManager::BoxManager()
    : manager_(new PluginManager<BoxedObject> ("csapex::BoxedObject"))
{
    manager_->loaded.connect(loaded);
}

namespace {
bool compare (SelectorProxy::Ptr a, SelectorProxy::Ptr b) {
    const std::string& as = BoxManager::stripNamespace(a->getType());
    const std::string& bs = BoxManager::stripNamespace(b->getType());
    return as.compare(bs) < 0;
}
}

void BoxManager::stop()
{
    if(manager_) {
        delete manager_;
        manager_ = NULL;
    }
}

void BoxManager::reset()
{
    uuids.clear();
}

BoxManager::~BoxManager()
{
    stop();
}

void BoxManager::reload()
{
    manager_->reload();

    Tag::createIfNotExists("General");

    foreach(SelectorProxy::Ptr p, available_elements_prototypes) {
        foreach(const Tag& tag, p->getTags()) {
            map[tag].push_back(p);
            tags.insert(tag);
        }
    }

    foreach(const Tag& cat, tags) {
        std::sort(map[cat].begin(), map[cat].end(), compare);
    }

    SelectorProxy::Ptr meta(new SelectorProxyImp<SubGraph>("::meta", ""));
    csapex::SelectorProxy::registerProxy(meta);

}

void BoxManager::insertAvailableBoxedObjects(QMenu* menu)
{
    if(!manager_->pluginsLoaded()) {
        manager_->reload();
    }

    foreach(const Tag& tag, tags) {
        QMenu* submenu = new QMenu(tag.getName().c_str());
        menu->addMenu(submenu);

        foreach(const SelectorProxy::Ptr& proxy, map[tag]) {
            QIcon icon = proxy->getIcon();
            QAction* action = new QAction(stripNamespace(proxy->getType()).c_str(), submenu);
            action->setData(QString(proxy->getType().c_str()));
            if(!icon.isNull()) {
                action->setIcon(icon);
                action->setIconVisibleInMenu(true);
            }
            action->setToolTip(proxy->getDescription().c_str());
            submenu->addAction(action);
        }
    }

    menu->menuAction()->setIconVisibleInMenu(true);

}

void BoxManager::insertAvailableBoxedObjects(QTreeWidget* tree)
{
    if(!manager_->pluginsLoaded()) {
        manager_->reload();
    }

    tree->setHeaderHidden(true);
    tree->setDragEnabled(true);

    foreach(const Tag& tag, tags) {

        QTreeWidgetItem* submenu = new QTreeWidgetItem;
        submenu->setText(0, tag.getName().c_str());
        tree->addTopLevelItem(submenu);

        foreach(const SelectorProxy::Ptr& proxy, map[tag]) {
            QIcon icon = proxy->getIcon();
            std::string name = stripNamespace(proxy->getType());

            QTreeWidgetItem* child = new QTreeWidgetItem;
            child->setToolTip(0, proxy->getDescription().c_str());
            child->setIcon(0, icon);
            child->setText(0, name.c_str());
            child->setData(0, Qt::UserRole, Box::MIME);
            child->setData(0, Qt::UserRole + 1, proxy->getType().c_str());

            submenu->addChild(child);
        }
    }


    QTreeWidgetItem* meta = new QTreeWidgetItem;

    QTreeWidgetItem* new_meta = new QTreeWidgetItem;
    new_meta->setText(0, "new meta box");
    new_meta->setData(0, Qt::UserRole, BoxGroup::MIME);

    meta->addChild(new_meta);
    meta->setText(0, "Meta Boxes");

    tree->addTopLevelItem(meta);
}

void BoxManager::register_box_type(SelectorProxy::Ptr provider)
{
    available_elements_prototypes.push_back(provider);
}

void BoxManager::startPlacingMetaBox(QWidget* parent, const QPoint& offset)
{
    std::cout << "meta" << std::endl;

    startPlacingBox(parent, "::meta", offset);
}


void BoxManager::startPlacingBox(QWidget* parent, const std::string &type, const QPoint& offset)
{
    foreach(SelectorProxy::Ptr p, available_elements_prototypes) {
        if(p->getType() == type) {
            p->startObjectPositioning(parent, p, offset);
            return;
        }
    }
}

std::string BoxManager::stripNamespace(const std::string &name)
{
    size_t from = name.find_first_of("::");
    return name.substr(from != name.npos ? from + 2 : 0);
}

Box::Ptr BoxManager::makeBox(QPoint pos, const std::string& target_type, const std::string& uuid)
{
    std::string type = target_type;
    if(type.find_first_of(" ") != type.npos) {
        std::cout << "warning: type '" << type << "' contains spaces, stripping them!" << std::endl;
        while(type.find(" ") != type.npos) {
            type.replace(type.find(" "), 1, "");
        }
    }


    std::string uuid_ = uuid;
    BOOST_FOREACH(SelectorProxy::Ptr p, available_elements_prototypes) {
        if(p->getType() == type) {
            if(uuid_.empty()) {
                uuid_ = makeUUID(type);
            }
            return p->create(pos, type, uuid);
        }
    }

    std::cout << "warning: cannot make box, type '" << type << "' is unknown, trying different namespace" << std::endl;

    std::string type_wo_ns = stripNamespace(type);

    BOOST_FOREACH(SelectorProxy::Ptr p, available_elements_prototypes) {
        std::string p_type_wo_ns = stripNamespace(p->getType());

        if(p_type_wo_ns == type_wo_ns) {
            if(uuid_.empty()) {
                uuid_ = makeUUID(type);
            }
            std::cout << "found a match: '" << type << " == " << p->getType() << std::endl;
            return p->create(pos, p->getType(), uuid);
        }
    }

    std::cerr << "error: cannot make box, type '" << type << "' is unknown\navailable:\n";
    BOOST_FOREACH(SelectorProxy::Ptr p, available_elements_prototypes) {
        std::cerr << p->getType() << '\n';
    }
    std::cerr << std::endl;
    return Box::NullPtr;
}

SelectorProxy::Ptr BoxManager::getSelector(const std::string &type)
{
    BOOST_FOREACH(SelectorProxy::Ptr p, available_elements_prototypes) {
        if(p->getType() == type) {
            return p;
        }
    }

    return SelectorProxy::NullPtr;
}


void BoxManager::setContainer(QWidget *c)
{
    container_ = c;
}

QWidget* BoxManager::container()
{
    return container_;
}

std::string BoxManager::makeUUID(const std::string& name)
{
    int& last_id = uuids[name];
    ++last_id;

    std::stringstream ss;
    ss << name << "_" << last_id;

    return ss.str();
}
