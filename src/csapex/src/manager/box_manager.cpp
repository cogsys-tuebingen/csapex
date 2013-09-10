/// HEADER
#include <csapex/manager/box_manager.h>

/// COMPONENT
#include <csapex/model/boxed_object_constructor.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_box.h>
#include <csapex/model/box.h>
#include <csapex/model/box_group.h>
#include <csapex/model/graph.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QApplication>
#include <QTreeWidget>
#include <stack>
#include <QDrag>
#include <qmime.h>

using namespace csapex;

BoxManager::BoxManager()
    : manager_(new PluginManager<BoxedObject> ("csapex::BoxedObject"))
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

    Tag::createIfNotExists("General");

    foreach(BoxedObjectConstructor::Ptr p, available_elements_prototypes) {
        foreach(const Tag& tag, p->getTags()) {
            map[tag].push_back(p);
            tags.insert(tag);
        }
    }

    foreach(const Tag& cat, tags) {
        std::sort(map[cat].begin(), map[cat].end(), compare);
    }
}

void BoxManager::insertAvailableBoxedObjects(QMenu* menu)
{
    if(!manager_->pluginsLoaded()) {
        manager_->reload();
    }

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
    if(!manager_->pluginsLoaded()) {
        manager_->reload();
    }

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

void BoxManager::register_box_type(BoxedObjectConstructor::Ptr provider)
{
    available_elements_prototypes.push_back(provider);
}

void BoxManager::startPlacingMetaBox(QWidget* parent, const QPoint& offset)
{
    std::cout << "meta" << std::endl;

    startPlacingBox(parent, "::meta", offset);
}

namespace {
QPixmap createPixmap(const std::string& type, const BoxedObjectPtr& content)
{
    csapex::Box::Ptr object;

    if(type == "::meta") {
        object.reset(new csapex::BoxGroup(content, ""));
    } else {
        object.reset(new csapex::Box(content, ""));
    }

    object->setObjectName(type.c_str());
    object->setLabel(type);
    object->setType(type);
    object->init(QPoint(0,0));
    object->getContent()->setTypeName(type);

    return QPixmap::grabWidget(object.get());
}
}

void BoxManager::startPlacingBox(QWidget* parent, const std::string &type, const QPoint& offset, const std::string& template_)
{
    BoxedObject::Ptr content;
    if(type == "::meta") {
        content.reset(new NullBoxedObject);
    } else {
        foreach(BoxedObjectConstructor::Ptr p, available_elements_prototypes) {
            if(p->getType() == type) {
                content = p->makeContent();
            }
        }
    }

    if(content) {
        QDrag* drag = new QDrag(parent);
        QMimeData* mimeData = new QMimeData;

        if(type == "::meta") {
            mimeData->setData(Template::MIME, template_.c_str());
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
    size_t from = name.find_first_of("::");
    return name.substr(from != name.npos ? from + 2 : 0);
}

Box::Ptr BoxManager::make(BoxedObject::Ptr content, QPoint pos, const std::string uuid, const std::string type)
{
    csapex::Box::Ptr object;

    if(type == "::meta") {
        object.reset(new csapex::BoxGroup(content, uuid));
    } else {
        object.reset(new csapex::Box(content, uuid));
    }

    object->setObjectName(uuid.c_str());
    object->setType(type);

    if(type != "::meta") {
        object->getContent()->setTypeName(type);
    }
    object->init(pos);

    return object;
}

Box::Ptr BoxManager::makeBox(QPoint pos, const std::string& target_type, const std::string& uuid)
{
    assert(!uuid.empty());

    if(target_type == "::meta") {
        return make(BoxedObject::Ptr(new NullBoxedObject), pos, uuid, target_type);
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
            return make(p->makeContent(), pos, uuid, type);
        }
    }

    std::cout << "warning: cannot make box, type '" << type << "' is unknown, trying different namespace" << std::endl;

    std::string type_wo_ns = stripNamespace(type);

    BOOST_FOREACH(BoxedObjectConstructor::Ptr p, available_elements_prototypes) {
        std::string p_type_wo_ns = stripNamespace(p->getType());

        if(p_type_wo_ns == type_wo_ns) {
            std::cout << "found a match: '" << type << " == " << p->getType() << std::endl;
            return make(p->makeContent(), pos, uuid, type);
        }
    }

    std::cerr << "error: cannot make box, type '" << type << "' is unknown\navailable:\n";
    BOOST_FOREACH(BoxedObjectConstructor::Ptr p, available_elements_prototypes) {
        std::cerr << p->getType() << '\n';
    }
    std::cerr << std::endl;
    return Box::NullPtr;
}

BoxedObjectConstructor::Ptr BoxManager::getSelector(const std::string &type)
{
    BOOST_FOREACH(BoxedObjectConstructor::Ptr p, available_elements_prototypes) {
        if(p->getType() == type) {
            return p;
        }
    }

    return BoxedObjectConstructor::NullPtr;
}


void BoxManager::setContainer(QWidget *c)
{
    container_ = c;
}

QWidget* BoxManager::container()
{
    return container_;
}
