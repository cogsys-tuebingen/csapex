/// HEADER
#include <csapex/view/widget_controller.h>

/// PROJECT
#include <csapex/command/delete_connection.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/move_box.h>
#include <csapex/command/move_fulcrum.h>
#include <csapex/model/connectable.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_factory.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/tag.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/trigger.h>
#include <csapex/utility/movable_graphics_proxy_widget.h>
#include <csapex/view/box.h>
#include <csapex/view/default_node_adapter.h>
#include <csapex/view/designer.h>
#include <csapex/view/port.h>
#include <csapex/view/node_adapter_factory.h>
#include <csapex/view/designer_view.h>
#include <csapex/core/settings.h>
#include <utils_param/parameter_factory.h>
#include <csapex/view/designer_scene.h>

/// SYSTEM
#include <QApplication>
#include <QTreeWidget>
#include <QDrag>
#include <QMimeData>
#include <QStandardItemModel>

using namespace csapex;

WidgetController::WidgetController(Settings& settings, Graph::Ptr graph, NodeFactory* node_factory, NodeAdapterFactory* node_adapter_factory)
    : graph_(graph), settings_(settings), node_factory_(node_factory), node_adapter_factory_(node_adapter_factory), designer_(NULL), tooltip_view_(NULL)
{
    if(settings_.knows("grid-lock")) {
        enableGridLock(settings_.get<bool>("grid-lock"));
    }
}

WidgetController::~WidgetController()
{
    delete tooltip_view_;
}

NodeBox* WidgetController::getBox(const UUID &node_id)
{
    boost::unordered_map<UUID, NodeBox*, UUID::Hasher>::const_iterator pos = box_map_.find(node_id);
    if(pos == box_map_.end()) {
        return NULL;
    }

    return pos->second;
}

MovableGraphicsProxyWidget* WidgetController::getProxy(const UUID &node_id)
{
    boost::unordered_map<UUID, MovableGraphicsProxyWidget*, UUID::Hasher>::const_iterator pos = proxy_map_.find(node_id);
    if(pos == proxy_map_.end()) {
        return NULL;
    }

    return pos->second;
}

Port* WidgetController::getPort(const UUID &connector_id)
{
    boost::unordered_map<UUID, Port*, UUID::Hasher>::const_iterator pos = port_map_.find(connector_id);
    if(pos == port_map_.end()) {
        return NULL;
    }

    return pos->second;
}

Port* WidgetController::getPort(const Connectable* connectable)
{
    boost::unordered_map<UUID, Port*, UUID::Hasher>::const_iterator pos = port_map_.find(connectable->getUUID());
    if(pos == port_map_.end()) {
        return NULL;
    }

    return pos->second;
}

Graph::Ptr WidgetController::getGraph()
{
    return graph_;
}

NodeFactory* WidgetController::getNodeFactory()
{
    return node_factory_;
}

void WidgetController::hideTooltipView()
{
    if(tooltip_view_) {
        tooltip_view_->hide();
        tooltip_view_->deleteLater();
        tooltip_view_ = NULL;
    }
}

QGraphicsView* WidgetController::getTooltipView(const std::string& title)
{
    if(!tooltip_view_) {
        tooltip_view_ = new QGraphicsView;
        designer_->getDesignerView()->designerScene()->addWidget(tooltip_view_);
        tooltip_view_->setWindowTitle(QString::fromStdString(title));
        tooltip_view_->setScene(new QGraphicsScene);
        tooltip_view_->setHidden(true);
    }

    return tooltip_view_;
}

void WidgetController::setDesigner(Designer *designer)
{
    designer_ = designer;
}

CommandDispatcher* WidgetController::getCommandDispatcher() const
{
    return dispatcher_;
}

void WidgetController::setCommandDispatcher(CommandDispatcher* dispatcher)
{
    dispatcher_ = dispatcher;
}

void WidgetController::setStyleSheet(const QString &str)
{
    style_sheet_ = str;
    designer_->overwriteStyleSheet(style_sheet_);
}

void WidgetController::startPlacingBox(QWidget *parent, const std::string &type, NodeStatePtr state, const QPoint &offset)
{
    NodeConstructor::Ptr c = node_factory_->getConstructor(type);
    NodeWorker::Ptr content = c->makePrototype();

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

    NodeBox::Ptr object(new NodeBox(settings_, content, NodeAdapter::Ptr(new DefaultNodeAdapter(content.get(), this)), c->getIcon()));

    object->setStyleSheet(style_sheet_);
    object->construct();
    object->setObjectName(content->getType().c_str());
    object->setLabel(type);

    drag->setPixmap(object->grab());
    drag->setHotSpot(-offset);
    drag->exec();
}

DesignerView* WidgetController::getDesignerView()
{
    return designer_->getDesignerView();
}

DesignerScene* WidgetController::getDesignerScene()
{
    return designer_->getDesignerView()->designerScene();
}

void WidgetController::nodeAdded(NodeWorkerPtr node_worker)
{
    if(designer_) {
        std::string type = node_worker->getType();

        NodeAdapter::Ptr adapter = node_adapter_factory_->makeNodeAdapter(node_worker.get(), this);

        QIcon icon = node_factory_->getConstructor(type)->getIcon();

        NodeBox* box = new NodeBox(settings_, node_worker, adapter, icon);

        box_map_[node_worker->getUUID()] = box;
        proxy_map_[node_worker->getUUID()] = new MovableGraphicsProxyWidget(box, designer_->getDesignerView(), this);

        box->construct();

        designer_->addBox(box);

        // add existing connectors
        Q_FOREACH(Input* input, node_worker->getMessageInputs()) {
            connectorMessageAdded(input);
        }
        Q_FOREACH(Output* output, node_worker->getMessageOutputs()) {
            connectorMessageAdded(output);
        }
        Q_FOREACH(Slot* slot, node_worker->getSlots()) {
            connectorSignalAdded(slot);
        }
        Q_FOREACH(Trigger* trigger, node_worker->getTriggers()) {
            connectorSignalAdded(trigger);
        }

        // subscribe to coming connectors
        QObject::connect(node_worker.get(), SIGNAL(connectorCreated(Connectable*)), this, SLOT(connectorCreated(Connectable*)));
        QObject::connect(node_worker.get(), SIGNAL(connectorRemoved(Connectable*)), this, SLOT(connectorRemoved(Connectable*)));

        Q_EMIT boxAdded(box);
    }
}

void WidgetController::nodeRemoved(NodeWorkerPtr node_worker)
{
    if(designer_) {
        UUID node_uuid = node_worker->getUUID();
        NodeBox* box = getBox(node_uuid);
        box->stop();

        box_map_.erase(box_map_.find(node_uuid));

        designer_->removeBox(box);
    }
}

void WidgetController::connectorCreated(Connectable* connector)
{
    // TODO: dirty...
    if(dynamic_cast<Slot*> (connector) || dynamic_cast<Trigger*>(connector)) {
        connectorSignalAdded(connector);
    } else {
        connectorMessageAdded(connector);
    }
}

void WidgetController::connectorRemoved(Connectable *connector)
{
    // TODO: dirty...
    if(dynamic_cast<Slot*> (connector) || dynamic_cast<Trigger*>(connector)) {
        connectorSignalRemoved(connector);
    } else {
        connectorMessageRemoved(connector);
    }
}

void WidgetController::connectorMessageAdded(Connectable* connector)
{
    UUID parent_uuid = connector->getUUID().parentUUID();
    NodeBox* box = getBox(parent_uuid);
    QBoxLayout* layout = connector->isInput() ? box->getInputLayout() : box->getOutputLayout();

    createPort(connector, box, layout);
}

void WidgetController::connectorMessageRemoved(Connectable *connector)
{
    if(designer_) {
        boost::unordered_map<UUID, Port*, UUID::Hasher>::iterator it = port_map_.find(connector->getUUID());
        if(it != port_map_.end()) {
            port_map_.erase(it);
        }
    }
}

void WidgetController::connectorSignalAdded(Connectable *connector)
{
    UUID parent_uuid = connector->getUUID().parentUUID();
    NodeBox* box = getBox(parent_uuid);
    QBoxLayout* layout = dynamic_cast<Trigger*>(connector) ? box->getTriggerLayout() : box->getSlotLayout();

    createPort(connector, box, layout);
}

void WidgetController::connectorSignalRemoved(Connectable *connector)
{
    connectorMessageRemoved(connector);
}

Port* WidgetController::createPort(Connectable* connector, NodeBox* box, QBoxLayout* layout)
{
    if(designer_) {
        Port* port = new Port(dispatcher_, this, connector);

        if(box) {
            port->setFlipped(box->isFlipped());
            port->setMinimizedSize(box->isMinimizedSize());

            QObject::connect(box, SIGNAL(minimized(bool)), port, SLOT(setMinimizedSize(bool)));
            QObject::connect(box, SIGNAL(flipped(bool)), port, SLOT(setFlipped(bool)));
        }

        insertPort(layout, port);

        return port;
    }

    return NULL;
}


void WidgetController::insertPort(QLayout* layout, Port* port)
{
    port_map_[port->getAdaptee()->getUUID()] = port;

    layout->addWidget(port);
}

void WidgetController::foreachBox(boost::function<void (NodeBox*)> f, boost::function<bool (NodeBox*)> pred)
{
    Q_FOREACH(NodeWorker::Ptr n, graph_->nodes_) {
        NodeBox* b = getBox(n->getUUID());
        if(pred(b)) {
            f(b);
        }
    }
}




void WidgetController::insertAvailableNodeTypes(QMenu* menu)
{
    std::map<TagPtr, std::vector<NodeConstructor::Ptr> > tags = node_factory_->getTagMap();

    typedef std::pair<TagPtr, std::vector<NodeConstructor::Ptr> > PAIR;
    Q_FOREACH(const PAIR& pair, tags) {
        const TagPtr& tag = pair.first;
        const std::vector<NodeConstructor::Ptr>& constructors = pair.second;

        QMenu* submenu = new QMenu(tag->getName().c_str());
        menu->addMenu(submenu);

        Q_FOREACH(const NodeConstructor::Ptr& proxy, constructors) {
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

void WidgetController::insertAvailableNodeTypes(QTreeWidget* tree)
{
    std::map<TagPtr, std::vector<NodeConstructor::Ptr> > tags = node_factory_->getTagMap();

    tree->setDragEnabled(true);

    typedef std::pair<TagPtr, std::vector<NodeConstructor::Ptr> > PAIR;
    Q_FOREACH(const PAIR& pair, tags) {
        const TagPtr& tag = pair.first;
        const std::vector<NodeConstructor::Ptr>& constructors = pair.second;

        QTreeWidgetItem* submenu = new QTreeWidgetItem;
        submenu->setText(0, tag->getName().c_str());
        tree->addTopLevelItem(submenu);

        Q_FOREACH(const NodeConstructor::Ptr& proxy, constructors) {
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

QAbstractItemModel* WidgetController::listAvailableNodeTypes()
{
    QStandardItemModel* model = new QStandardItemModel;

    Q_FOREACH(const NodeConstructor::Ptr& proxy, node_factory_->getConstructors()) {
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

bool WidgetController::isGridLockEnabled() const
{
    return settings_.get<bool>("grid-lock", false);
}

void WidgetController::enableGridLock(bool enabled)
{
    if(!settings_.knows("grid-lock")) {
        settings_.add(param::ParameterFactory::declareBool("grid-lock", enabled));
    }

    settings_.set("grid-lock", enabled);

    Q_EMIT gridLockEnabled(enabled);
}
