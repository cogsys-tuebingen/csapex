/// HEADER
#include <csapex/view/widget_controller.h>

/// PROJECT
#include <csapex/command/delete_connection.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/move_box.h>
#include <csapex/command/move_fulcrum.h>
#include <csapex/model/connectable.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_worker.h>
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
#include <unordered_map>
#include <QPointer>

using namespace csapex;

class WidgetController::Impl
{
public:
    Impl()
        : tooltip_view_(nullptr)
    {
    }

    ~Impl()
    {
        delete tooltip_view_;
    }

public:
    std::unordered_map<UUID, QPointer<NodeBox>, UUID::Hasher> box_map_;
    std::unordered_map<UUID, MovableGraphicsProxyWidget*, UUID::Hasher> proxy_map_;
    std::unordered_map<UUID, QPointer<Port>, UUID::Hasher> port_map_;

    QString style_sheet_;

    QGraphicsView* tooltip_view_;
};

WidgetController::WidgetController(Settings& settings, CommandDispatcher& dispatcher, GraphWorker::Ptr graph, NodeFactory* node_factory, NodeAdapterFactory* node_adapter_factory)
    : graph_(graph), dispatcher_(dispatcher), settings_(settings), node_factory_(node_factory), node_adapter_factory_(node_adapter_factory), designer_(nullptr),
      pimpl(new Impl)
{
    if(settings_.knows("grid-lock")) {
        enableGridLock(settings_.get<bool>("grid-lock"));
    }
}

WidgetController::~WidgetController()
{
    for(auto remaining_connections : connections_) {
        remaining_connections.disconnect();
    }
}

NodeBox* WidgetController::getBox(const UUID &node_id)
{
    std::unordered_map<UUID, QPointer<NodeBox>, UUID::Hasher>::const_iterator pos = pimpl->box_map_.find(node_id);
    if(pos == pimpl->box_map_.end()) {
        return nullptr;
    }

    return pos->second;
}

MovableGraphicsProxyWidget* WidgetController::getProxy(const UUID &node_id)
{
    std::unordered_map<UUID, MovableGraphicsProxyWidget*, UUID::Hasher>::const_iterator pos = pimpl->proxy_map_.find(node_id);
    if(pos == pimpl->proxy_map_.end()) {
        return nullptr;
    }

    return pos->second;
}

Port* WidgetController::getPort(const UUID &connector_id)
{
    std::unordered_map<UUID, QPointer<Port>, UUID::Hasher>::const_iterator pos = pimpl->port_map_.find(connector_id);
    if(pos == pimpl->port_map_.end()) {
        return nullptr;
    }

    return pos->second;
}

Port* WidgetController::getPort(const Connectable* connectable)
{
    std::unordered_map<UUID, QPointer<Port>, UUID::Hasher>::const_iterator pos = pimpl->port_map_.find(connectable->getUUID());
    if(pos == pimpl->port_map_.end()) {
        return nullptr;
    }

    return pos->second;
}

GraphWorker::Ptr WidgetController::getGraph()
{
    return graph_;
}

NodeFactory* WidgetController::getNodeFactory()
{
    return node_factory_;
}

void WidgetController::hideTooltipView()
{
    if(pimpl->tooltip_view_) {
        pimpl->tooltip_view_->hide();
        pimpl->tooltip_view_->deleteLater();
        pimpl->tooltip_view_ = nullptr;
    }
}

QGraphicsView* WidgetController::getTooltipView(const std::string& title)
{
    if(!pimpl->tooltip_view_) {
        pimpl->tooltip_view_ = new QGraphicsView;
        designer_->getDesignerView()->designerScene()->addWidget(pimpl->tooltip_view_);
        pimpl->tooltip_view_->setWindowTitle(QString::fromStdString(title));
        pimpl->tooltip_view_->setScene(new QGraphicsScene);
        pimpl->tooltip_view_->setHidden(true);
    }

    return pimpl->tooltip_view_;
}

void WidgetController::setDesigner(Designer *designer)
{
    designer_ = designer;
}

CommandDispatcher* WidgetController::getCommandDispatcher() const
{
    return &dispatcher_;
}

void WidgetController::setStyleSheet(const QString &str)
{
    pimpl->style_sheet_ = str;
    designer_->overwriteStyleSheet(pimpl->style_sheet_);
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

    NodeBox::Ptr object(new NodeBox(settings_, content,
                                    NodeAdapter::Ptr(std::make_shared<DefaultNodeAdapter>(content, this)),
                                    QIcon(QString::fromStdString(c->getIcon()))));

    object->setStyleSheet(pimpl->style_sheet_);
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

        NodeAdapter::Ptr adapter = node_adapter_factory_->makeNodeAdapter(node_worker, this);

        QIcon icon = QIcon(QString::fromStdString(node_factory_->getConstructor(type)->getIcon()));

        NodeBox* box = new NodeBox(settings_, node_worker, adapter, icon);

        pimpl->box_map_[node_worker->getUUID()] = box;
        pimpl->proxy_map_[node_worker->getUUID()] = new MovableGraphicsProxyWidget(box, designer_->getDesignerView(), this);

        box->construct();

        designer_->addBox(box);

        // add existing connectors
        for(Input* input : node_worker->getMessageInputs()) {
            connectorMessageAdded(input);
        }
        for(Output* output : node_worker->getMessageOutputs()) {
            connectorMessageAdded(output);
        }
        for(Slot* slot : node_worker->getSlots()) {
            connectorSignalAdded(slot);
        }
        for(Trigger* trigger : node_worker->getTriggers()) {
            connectorSignalAdded(trigger);
        }

        // subscribe to coming connectors
        auto c1 = node_worker->connectorCreated.connect([this](ConnectablePtr c) { connectorCreated(c.get()); });
        connections_.push_back(c1);
        auto c2 = node_worker->connectorRemoved.connect([this](ConnectablePtr c) { connectorRemoved(c.get()); });
        connections_.push_back(c2);

        Q_EMIT boxAdded(box);
    }
}

void WidgetController::nodeRemoved(NodeWorkerPtr node_worker)
{
    if(designer_) {
        UUID node_uuid = node_worker->getUUID();
        NodeBox* box = getBox(node_uuid);
        box->stop();

        pimpl->box_map_.erase(pimpl->box_map_.find(node_uuid));

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
        auto it = pimpl->port_map_.find(connector->getUUID());
        if(it != pimpl->port_map_.end()) {
            pimpl->port_map_.erase(it);
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
        Port* port = new Port(&dispatcher_, this, connector);

        if(box) {
            port->setFlipped(box->isFlipped());
            port->setMinimizedSize(box->isMinimizedSize());

            QObject::connect(box, SIGNAL(minimized(bool)), port, SLOT(setMinimizedSize(bool)));
            QObject::connect(box, SIGNAL(flipped(bool)), port, SLOT(setFlipped(bool)));
        }

        QObject::connect(port, SIGNAL(destroyed(QObject*)), this, SLOT(portDestroyed(QObject*)));

        insertPort(layout, port);

        return port;
    }

    return nullptr;
}

void WidgetController::portDestroyed(QObject *o)
{
    Port* p = dynamic_cast<Port*>(o);
    if(p) {
        for(auto it = pimpl->port_map_.begin(); it != pimpl->port_map_.end(); ) {
            if(it->second == p) {
                it = pimpl->port_map_.erase(it);
            } else {
                ++it;
            }
        }
    }
}


void WidgetController::insertPort(QLayout* layout, Port* port)
{
    pimpl->port_map_[port->getAdaptee()->getUUID()] = port;

    layout->addWidget(port);
}



void WidgetController::insertAvailableNodeTypes(QMenu* menu)
{
    auto tags = node_factory_->getTagMap();

    for(const auto& pair : tags) {
        const std::string& tag = pair.first;
        const std::vector<NodeConstructor::Ptr>& constructors = pair.second;

        QMenu* submenu = new QMenu(QString::fromStdString(tag));
        menu->addMenu(submenu);

        for(const NodeConstructor::Ptr& proxy : constructors) {
            QIcon icon = QIcon(QString::fromStdString(proxy->getIcon()));
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
    auto tags = node_factory_->getTagMap();

    tree->setDragEnabled(true);

    for(const auto& pair : tags) {
        const std::string& tag = pair.first;
        const std::vector<NodeConstructor::Ptr>& constructors = pair.second;

        QTreeWidgetItem* submenu = new QTreeWidgetItem;
        submenu->setText(0, QString::fromStdString(tag));
        tree->addTopLevelItem(submenu);

        for(const NodeConstructor::Ptr& proxy : constructors) {
            QIcon icon = QIcon(QString::fromStdString(proxy->getIcon()));
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

    for(const NodeConstructor::Ptr& proxy : node_factory_->getConstructors()) {
        QString name = QString::fromStdString(UUID::stripNamespace(proxy->getType()));
        QString descr(proxy->getDescription().c_str());
        QString type(proxy->getType().c_str());

        QStringList tags;
        for(const Tag::Ptr& tag : proxy->getTags()) {
            tags << tag->getName().c_str();
        }

        QStandardItem* item = new QStandardItem(QIcon(QString::fromStdString(proxy->getIcon())), type);
        item->setData(type, Qt::UserRole);
        item->setData(descr, Qt::UserRole + 1);
        item->setData(name, Qt::UserRole + 2);
        item->setData(tags, Qt::UserRole + 3);

        model->appendRow(item);
    }

    model->sort(0);

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
/// MOC
#include "../../include/csapex/view/moc_widget_controller.cpp"
