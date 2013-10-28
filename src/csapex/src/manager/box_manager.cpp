/// HEADER
#include <csapex/manager/box_manager.h>

/// COMPONENT
#include <csapex/model/boxed_object_constructor.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_node.h>
#include <csapex/model/box.h>
#include <csapex/model/group.h>
#include <csapex/model/graph.h>
#include <utils_plugin/plugin_manager.hpp>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QApplication>
#include <QTreeWidget>
#include <stack>
#include <QDrag>
#include <qmime.h>

using namespace csapex;

BoxManager::BoxManager()
    : manager_(new PluginManager<BoxedObject> ("csapex::BoxedObject")), dirty_(false)
{
    manager_->loaded.connect(loaded);
}

namespace {
bool compare (BoxedObjectConstructor::Ptr a, BoxedObjectConstructor::Ptr b) {
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
}

BoxManager::~BoxManager()
{
    stop();
}

void BoxManager::reload()
{
    manager_->reload();
    rebuildPrototypes();

    rebuildMap();
}

void BoxManager::rebuildPrototypes()
{
    available_elements_prototypes.clear();

    typedef std::pair<std::string, DefaultConstructor<BoxedObject> > PAIR;
    foreach(const PAIR& p, manager_->availableClasses()) {
        csapex::BoxedObjectConstructor::Ptr constructor(new csapex::BoxedObjectConstructor(
                                                            p.second.getType(), p.second.getDescription(),
                                                            p.second));
        register_box_type(constructor, true);
    }
}

void BoxManager::rebuildMap()
{
    Tag::createIfNotExists("General");
    Tag general = Tag::get("General");

    map.clear();
    tags.clear();

    tags.insert(general);

    foreach(BoxedObjectConstructor::Ptr p, available_elements_prototypes) {
        bool has_tag = false;
        foreach(const Tag& tag, p->getTags()) {
            map[tag].push_back(p);
            tags.insert(tag);
            has_tag = true;
        }

        if(!has_tag) {
            map[general].push_back(p);
        }
    }

    foreach(const Tag& cat, tags) {
        std::sort(map[cat].begin(), map[cat].end(), compare);
    }

    dirty_ = false;
}

void BoxManager::ensureLoaded()
{
    if(!manager_->pluginsLoaded()) {
        manager_->reload();
        rebuildPrototypes();

        dirty_ = true;
    }

    if(dirty_) {
        rebuildMap();
    }
}

void BoxManager::insertAvailableBoxedObjects(QMenu* menu)
{
    ensureLoaded();

    foreach(const Tag& tag, tags) {
        QMenu* submenu = new QMenu(tag.getName().c_str());
        menu->addMenu(submenu);

        foreach(const BoxedObjectConstructor::Ptr& proxy, map[tag]) {
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
    ensureLoaded();

    tree->setHeaderHidden(true);
    tree->setDragEnabled(true);

    foreach(const Tag& tag, tags) {

        QTreeWidgetItem* submenu = new QTreeWidgetItem;
        submenu->setText(0, tag.getName().c_str());
        tree->addTopLevelItem(submenu);

        foreach(const BoxedObjectConstructor::Ptr& proxy, map[tag]) {
            QIcon icon = proxy->getIcon();
            std::string name = stripNamespace(proxy->getType());

            QTreeWidgetItem* child = new QTreeWidgetItem;
            child->setToolTip(0, (proxy->getType() + ": " + proxy->getDescription()).c_str());
            child->setIcon(0, icon);
            child->setText(0, name.c_str());
            child->setData(0, Qt::UserRole, Box::MIME);
            child->setData(0, Qt::UserRole + 1, proxy->getType().c_str());

            submenu->addChild(child);
        }
    }
}

void BoxManager::register_box_type(BoxedObjectConstructor::Ptr provider, bool suppress_signals)
{
    available_elements_prototypes.push_back(provider);
    dirty_ = true;

    if(!suppress_signals) {
        new_box_type();
    }
}

namespace {
QPixmap createPixmap(const std::string& label, const BoxedObjectPtr& content)
{
    csapex::Box::Ptr object;

    if(BoxManager::typeIsTemplate(content->getType())) {
        object.reset(new csapex::Group(""));
    } else {
        object.reset(new csapex::Box(content.get(), content));
    }

    object->setObjectName(content->getType().c_str());
    object->setLabel(label);

    return QPixmap::grabWidget(object.get());
}
}

bool BoxManager::typeIsTemplate(const std::string &type)
{
    return type.substr(0,12) == "::template::";
}

std::string BoxManager::getTemplateName(const std::string &type)
{
    assert(typeIsTemplate(type));
    return type.substr(12);
}

void BoxManager::startPlacingBox(QWidget* parent, const std::string &type, const QPoint& offset)
{
    bool is_template = BoxManager::typeIsTemplate(type);

    BoxedObject::Ptr content;
    if(is_template) {
        content.reset(new NullBoxedObject(type));
    } else {
        foreach(BoxedObjectConstructor::Ptr p, available_elements_prototypes) {
            if(p->getType() == type) {
                content = p->makePrototypeContent();
            }
        }
    }

    if(content) {
        QDrag* drag = new QDrag(parent);
        QMimeData* mimeData = new QMimeData;

        if(is_template) {
            mimeData->setData(Template::MIME, "");
        }
        mimeData->setData(Box::MIME, type.c_str());
        mimeData->setProperty("ox", offset.x());
        mimeData->setProperty("oy", offset.y());
        drag->setMimeData(mimeData);

        drag->setPixmap(createPixmap(type, content));
        drag->setHotSpot(-offset);
        drag->exec();
        return;
    }
}

std::string BoxManager::stripNamespace(const std::string &name)
{
    size_t from = name.rfind("::");
    return name.substr(from != name.npos ? from + 2 : 0);
}

Node::Ptr BoxManager::makeSingleNode(BoxedObjectConstructor::Ptr content, const std::string uuid)
{
    assert(!BoxManager::typeIsTemplate(content->getType()) && content->getType() != "::group");

    BoxedObject::Ptr bo = content->makeContent(uuid);

    return bo;
}

Node::Ptr BoxManager::makeTemplateNode(const std::string uuid, const std::string type)
{
    assert(BoxManager::typeIsTemplate(type) || type == "::group");

//    csapex::Group::Ptr group(new csapex::Group(type, uuid));

//    group->setObjectName(uuid.c_str());

//    return group;
    std::cerr << "WARNING: reimplement!" << std::endl;
    return Node::Ptr(new Node(""));
}

Node::Ptr BoxManager::makeNode(const std::string& target_type, const std::string& uuid)
{
    assert(!uuid.empty());


    if(BoxManager::typeIsTemplate(target_type) || target_type == "::group") {
        return makeTemplateNode(uuid, target_type);
    }

    std::string type = target_type;
    if(type.find_first_of(" ") != type.npos) {
        std::cout << "warning: type '" << type << "' contains spaces, stripping them!" << std::endl;
        while(type.find(" ") != type.npos) {
            type.replace(type.find(" "), 1, "");
        }
    }


    BOOST_FOREACH(BoxedObjectConstructor::Ptr p, available_elements_prototypes) {
        if(p->getType() == type) {
            return makeSingleNode(p, uuid);
        }
    }

    std::cout << "warning: cannot make box, type '" << type << "' is unknown, trying different namespace" << std::endl;

    std::string type_wo_ns = stripNamespace(type);

    BOOST_FOREACH(BoxedObjectConstructor::Ptr p, available_elements_prototypes) {
        std::string p_type_wo_ns = stripNamespace(p->getType());

        if(p_type_wo_ns == type_wo_ns) {
            std::cout << "found a match: '" << type << " == " << p->getType() << std::endl;
            return makeSingleNode(p, uuid);
        }
    }

    std::cerr << "error: cannot make box, type '" << type << "' is unknown\navailable:\n";
    BOOST_FOREACH(BoxedObjectConstructor::Ptr p, available_elements_prototypes) {
        std::cerr << p->getType() << '\n';
    }
    std::cerr << std::endl;
    return NodeNullPtr;
}

BoxedObjectConstructor::Ptr BoxManager::getSelector(const std::string &type)
{
    BOOST_FOREACH(BoxedObjectConstructor::Ptr p, available_elements_prototypes) {
        if(p->getType() == type) {
            return p;
        }
    }

    return BoxedObjectConstructorNullPtr;
}


void BoxManager::setContainer(QWidget *c)
{
    container_ = c;
}

QWidget* BoxManager::container()
{
    return container_;
}
