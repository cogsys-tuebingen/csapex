/// HEADER
#include <csapex/manager/box_manager.h>

/// COMPONENT
#include <csapex/command/delete_node.h>
#include <csapex/command/meta.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/tag.h>
#include <csapex/utility/uuid.h>
#include <csapex/view/box.h>
#include <csapex/view/default_node_adapter.h>
#include <csapex/utility/plugin_manager.hpp>
#include <csapex/view/widget_controller.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QApplication>
#include <QTreeWidget>
#include <stack>
#include <QDrag>
#include <qmime.h>
#include <QStandardItemModel>
#include <boost/algorithm/string.hpp>

using namespace csapex;

BoxManager::BoxManager()
    : node_manager_(new PluginManager<Node> ("csapex::Node")),
      node_adapter_manager_(new PluginManager<NodeAdapterBuilder> ("csapex::NodeAdapterBuilder")),
      dirty_(false)
{
    node_manager_->loaded.connect(loaded);
    node_adapter_manager_->loaded.connect(loaded);
}

namespace {
bool compare (NodeConstructor::Ptr a, NodeConstructor::Ptr b) {
    const std::string& as = UUID::stripNamespace(a->getType());
    const std::string& bs = UUID::stripNamespace(b->getType());
    return as.compare(bs) < 0;
}
}

void BoxManager::stop()
{
    delete node_manager_;
    node_manager_ = NULL;
    
    node_adapter_builders_.clear();
    
    delete node_adapter_manager_;
    node_adapter_manager_ = NULL;
}


void BoxManager::setStyleSheet(const QString &str)
{
    style_sheet_ = str;
}

BoxManager::~BoxManager()
{
    stop();
}

void BoxManager::reload()
{
    node_manager_->reload();
    node_adapter_manager_->reload();
    rebuildPrototypes();
    
    rebuildMap();
}

void BoxManager::rebuildPrototypes()
{
    available_elements_prototypes.clear();
    node_adapter_builders_.clear();
    
    typedef std::pair<std::string, DefaultConstructor<Node> > NODE_PAIR;
    Q_FOREACH(const NODE_PAIR& p, node_manager_->availableClasses()) {
        // convert tag list into vector
        std::vector<std::string> tokens;
        std::vector<Tag::Ptr> tags;

        std::string taglist = p.second.getTags();
        boost::algorithm::split(tokens, taglist, boost::is_any_of(",;"));

        for(std::vector<std::string>::const_iterator it = tokens.begin(); it != tokens.end(); ++it) {
            std::string str = boost::algorithm::trim_copy(*it);
            if(!str.empty()) {
                tags.push_back(Tag::get(str));
            }
        }

        if(tags.empty()) {
            tags.push_back(Tag::get("General"));
        }

        // make the constructor
        csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor(
                                                     *settings_, dispatcher_,
                                                     p.second.getType(), p.second.getDescription(),
                                                     p.second.getIcon(),
                                                     tags,
                                                     p.second));
        register_box_type(constructor, true);
    }
    
    typedef std::pair<std::string, DefaultConstructor<NodeAdapterBuilder> > ADAPTER_PAIR;
    Q_FOREACH(const ADAPTER_PAIR& p, node_adapter_manager_->availableClasses()) {
        NodeAdapterBuilder::Ptr builder = p.second.construct();
        node_adapter_builders_[builder->getWrappedType()] = builder;
    }
}

void BoxManager::rebuildMap()
{
    Tag::createIfNotExists("General");
    Tag::Ptr general = Tag::get("General");
    
    tag_map_.clear();
    tags_.clear();
    
    tags_.insert(general);
    
    for(std::vector<NodeConstructor::Ptr>::iterator
        it = available_elements_prototypes.begin();
        it != available_elements_prototypes.end();) {
        
        const NodeConstructor::Ptr& p = *it;
        
        try {
            bool has_tag = false;
            Q_FOREACH(const Tag::Ptr& tag, p->getTags()) {
                tag_map_[tag].push_back(p);
                tags_.insert(tag);
                has_tag = true;
            }
            
            if(!has_tag) {
                tag_map_[general].push_back(p);
            }
            
            ++it;
            
        } catch(const NodeConstructor::NodeConstructionException& e) {
            std::cerr << "warning: cannot load node: " << e.what() << std::endl;
            it = available_elements_prototypes.erase(it);
        }
    }
    
    Q_FOREACH(const Tag::Ptr& cat, tags_) {
        std::sort(tag_map_[cat].begin(), tag_map_[cat].end(), compare);
    }
    
    dirty_ = false;
}

void BoxManager::ensureLoaded()
{
    if(!node_manager_->pluginsLoaded()) {
        node_manager_->reload();
        node_adapter_manager_->reload();
        
        rebuildPrototypes();
        
        dirty_ = true;
    }
    
    if(dirty_) {
        rebuildMap();
    }
}

void BoxManager::insertAvailableNodeTypes(QMenu* menu)
{
    ensureLoaded();
    
    Q_FOREACH(const Tag::Ptr& tag, tags_) {
        QMenu* submenu = new QMenu(tag->getName().c_str());
        menu->addMenu(submenu);
        
        Q_FOREACH(const NodeConstructor::Ptr& proxy, tag_map_[tag]) {
            QIcon icon = proxy->getIcon();
            QAction* action = new QAction(UUID::stripNamespace(proxy->getType()).c_str(), submenu);
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

void BoxManager::insertAvailableNodeTypes(QTreeWidget* tree)
{
    ensureLoaded();
    
    tree->setDragEnabled(true);
    
    Q_FOREACH(const Tag::Ptr& tag, tags_) {
        
        QTreeWidgetItem* submenu = new QTreeWidgetItem;
        submenu->setText(0, tag->getName().c_str());
        tree->addTopLevelItem(submenu);
        
        Q_FOREACH(const NodeConstructor::Ptr& proxy, tag_map_[tag]) {
            QIcon icon = proxy->getIcon();
            std::string name = UUID::stripNamespace(proxy->getType());
            
            QTreeWidgetItem* child = new QTreeWidgetItem;
            child->setToolTip(0, (proxy->getType() + ": " + proxy->getDescription()).c_str());
            child->setIcon(0, icon);
            child->setText(0, name.c_str());
            child->setData(0, Qt::UserRole, NodeBox::MIME);
            child->setData(0, Qt::UserRole + 1, proxy->getType().c_str());
            
            submenu->addChild(child);
        }
    }
}


QAbstractItemModel* BoxManager::listAvailableNodeTypes()
{
    ensureLoaded();
    
    QStandardItemModel* model = new QStandardItemModel;//(types, 1);
    
    Q_FOREACH(const NodeConstructor::Ptr& proxy, available_elements_prototypes) {
        QString name = QString::fromStdString(UUID::stripNamespace(proxy->getType()));
        QString descr(proxy->getDescription().c_str());
        QString type(proxy->getType().c_str());
        
        QStringList tags;
        Q_FOREACH(const Tag::Ptr& tag, proxy->getTags()) {
            tags << tag->getName().c_str();
        }
        
        QStandardItem* item = new QStandardItem(proxy->getIcon(), type);
        item->setData(type, Qt::UserRole);
        item->setData(descr, Qt::UserRole + 1);
        item->setData(name, Qt::UserRole + 2);
        item->setData(tags, Qt::UserRole + 3);
        
        model->appendRow(item);
    }
    
    return model;
}
void BoxManager::register_box_type(NodeConstructor::Ptr provider, bool suppress_signals)
{
    available_elements_prototypes.push_back(provider);
    dirty_ = true;
    
    if(!suppress_signals) {
        new_box_type();
    }
}

bool BoxManager::isValidType(const std::string &type) const
{
    Q_FOREACH(NodeConstructor::Ptr p, available_elements_prototypes) {
        if(p->getType() == type) {
            return true;
        }
    }
    
    return false;
}

void BoxManager::startPlacingBox(QWidget* parent, const std::string &type, WidgetController* widget_ctrl, NodeStatePtr state, const QPoint& offset)
{
    NodeConstructor::Ptr c = getConstructor(type);
    Node::Ptr content = c->makePrototypeContent();

    QDrag* drag = new QDrag(parent);
    QMimeData* mimeData = new QMimeData;

    mimeData->setData(NodeBox::MIME, type.c_str());
    if(state) {
        QVariant payload = qVariantFromValue((void *) &state);
        mimeData->setProperty("state", payload);
    }
    mimeData->setProperty("ox", offset.x());
    mimeData->setProperty("oy", offset.y());
    drag->setMimeData(mimeData);


    NodeBox::Ptr object(new NodeBox(*settings_, widget_ctrl->getCommandDispatcher(), content, NodeAdapter::Ptr(new DefaultNodeAdapter(content.get(), widget_ctrl)), c->getIcon()));

    object->setStyleSheet(style_sheet_);
    object->construct();
    object->setObjectName(content->getType().c_str());
    object->setLabel(type);


    drag->setPixmap(QPixmap::grabWidget(object.get()));
    drag->setHotSpot(-offset);
    drag->exec();
}

Node::Ptr BoxManager::makeSingleNode(NodeConstructor::Ptr content, const UUID& uuid)
{
    
    Node::Ptr bo = content->makeContent(uuid);
    
    return bo;
}

NodeConstructor::Ptr BoxManager::getConstructor(const std::string &target_type)
{
    std::string type = target_type;
    if(type.find_first_of(" ") != type.npos) {
        std::cout << "warning: type '" << type << "' contains spaces, stripping them!" << std::endl;
        while(type.find(" ") != type.npos) {
            type.replace(type.find(" "), 1, "");
        }
    }

    BOOST_FOREACH(NodeConstructor::Ptr p, available_elements_prototypes) {
        if(p->getType() == type) {
            return p;
        }
    }

    std::cout << "warning: cannot make box, type '" << type << "' is unknown, trying different namespace" << std::endl;

    std::string type_wo_ns = UUID::stripNamespace(type);

    BOOST_FOREACH(NodeConstructor::Ptr p, available_elements_prototypes) {
        std::string p_type_wo_ns = UUID::stripNamespace(p->getType());

        if(p_type_wo_ns == type_wo_ns) {
            std::cout << "found a match: '" << type << " == " << p->getType() << std::endl;
            return p;
        }
    }

    return NodeConstructorNullPtr;
}

Node::Ptr BoxManager::makeNode(const std::string& target_type, const UUID& uuid)
{
    return makeNode(target_type, uuid, NodeStateNullPtr);
}

Node::Ptr BoxManager::makeNode(const std::string& target_type, const UUID& uuid, NodeStatePtr state)
{
    apex_assert_hard(!uuid.empty());

    NodeConstructorPtr p = getConstructor(target_type);
    if(p) {
        Node::Ptr result = makeSingleNode(p, uuid);

        if(state) {
            result->setNodeState(state);
        }

        return result;

    } else {
        std::cerr << "error: cannot make box, type '" << target_type << "' is unknown\navailable:\n";
        BOOST_FOREACH(NodeConstructor::Ptr p, available_elements_prototypes) {
            std::cerr << p->getType() << '\n';
        }
        std::cerr << std::endl;
        return NodeNullPtr;
    }
}

NodeBox* BoxManager::makeBox(NodePtr node, WidgetController* widget_ctrl)
{
    std::string type = node->getType();
    
    NodeBox* box;
    QIcon icon = getConstructor(type)->getIcon();
    if(node_adapter_builders_.find(type) != node_adapter_builders_.end()) {
        box = new NodeBox(*settings_, widget_ctrl->getCommandDispatcher(), node, node_adapter_builders_[type]->build(node, widget_ctrl), icon);
    } else {
        box = new NodeBox(*settings_, widget_ctrl->getCommandDispatcher(), node, NodeAdapter::Ptr(new DefaultNodeAdapter(node.get(), widget_ctrl)), icon);
    }
    box->construct();
    return box;
}
