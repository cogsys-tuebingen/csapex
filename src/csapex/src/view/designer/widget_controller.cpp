/// HEADER
#include <csapex/view/designer/widget_controller.h>

/// PROJECT
#include <csapex/command/delete_connection.h>
#include <csapex/command/disable_node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/core/settings.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/connectable.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/tag.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/trigger.h>
#include <csapex/view/designer/designer.h>
#include <csapex/view/designer/designer_view.h>
#include <csapex/view/node/box.h>
#include <csapex/view/node/default_node_adapter.h>
#include <csapex/view/node/node_adapter_factory.h>
#include <csapex/view/widgets/movable_graphics_proxy_widget.h>
#include <csapex/view/widgets/port.h>

/// SYSTEM
#include <QApplication>
#include <QThread>
#include <QDrag>
#include <QMimeData>
#include <unordered_map>
#include <QPointer>

using namespace csapex;

class WidgetController::Impl
{
public:
    Impl()
    {
    }

    ~Impl()
    {
    }


public:
    std::unordered_map<UUID, QPointer<NodeBox>, UUID::Hasher> box_map_;
    std::unordered_map<UUID, MovableGraphicsProxyWidget*, UUID::Hasher> proxy_map_;
    std::unordered_map<UUID, QPointer<Port>, UUID::Hasher> port_map_;

    QString style_sheet_;

    MessagePreviewWidget* preview_widget_;
};

WidgetController::WidgetController(Settings& settings, CommandDispatcher& dispatcher, GraphFacade::Ptr graph, NodeFactory* node_factory, NodeAdapterFactory* node_adapter_factory)
    : graph_(graph), dispatcher_(dispatcher), settings_(settings), node_factory_(node_factory), node_adapter_factory_(node_adapter_factory), designer_(nullptr),
      pimpl(new Impl)
{
    if(settings_.knows("grid-lock")) {
        enableGridLock(settings_.get<bool>("grid-lock"));
    }

    connect(this, SIGNAL(triggerConnectorCreated(ConnectablePtr)), this, SLOT(connectorCreated(ConnectablePtr)));
    connect(this, SIGNAL(triggerConnectorRemoved(ConnectablePtr)), this, SLOT(connectorRemoved(ConnectablePtr)));

    qRegisterMetaType < ConnectablePtr > ("ConnectablePtr");
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

GraphFacade::Ptr WidgetController::getGraph()
{
    return graph_;
}

NodeFactory* WidgetController::getNodeFactory()
{
    return node_factory_;
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
    NodeConstructorPtr c = node_factory_->getConstructor(type);
    NodeHandlePtr handle = c->makePrototype();

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

    NodeBox::Ptr object(new NodeBox(settings_, handle,
                                    NodeAdapter::Ptr(std::make_shared<DefaultNodeAdapter>(handle, this)),
                                    QIcon(QString::fromStdString(c->getIcon()))));

    object->setStyleSheet(pimpl->style_sheet_);
    object->construct();
    object->setObjectName(handle->getType().c_str());
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
        NodeHandlePtr node_handle = node_worker->getNodeHandle();
        std::string type = node_handle->getType();

        NodeAdapter::Ptr adapter = node_adapter_factory_->makeNodeAdapter(node_handle, this);

        QIcon icon = QIcon(QString::fromStdString(node_factory_->getConstructor(type)->getIcon()));

        NodeBox* box = new NodeBox(settings_, node_handle, node_worker, adapter, icon);

        pimpl->box_map_[node_handle->getUUID()] = box;
        pimpl->proxy_map_[node_handle->getUUID()] = new MovableGraphicsProxyWidget(box, designer_->getDesignerView(), this);

        box->construct();

        designer_->addBox(box);

        // add existing connectors
        for(auto input : node_handle->getAllInputs()) {
            connectorMessageAdded(input);
        }
        for(auto output : node_handle->getAllOutputs()) {
            connectorMessageAdded(output);
        }
        for(auto slot : node_handle->getAllSlots()) {
            connectorSignalAdded(slot);
        }
        for(auto trigger : node_handle->getAllTriggers()) {
            connectorSignalAdded(trigger);
        }

        // subscribe to coming connectors
        auto c1 = node_handle->connectorCreated.connect([this](ConnectablePtr c) { triggerConnectorCreated(c); });
        connections_.push_back(c1);
        auto c2 = node_handle->connectorRemoved.connect([this](ConnectablePtr c) { triggerConnectorRemoved(c); });
        connections_.push_back(c2);


        UUID uuid = node_handle->getUUID();
        QObject::connect(box, &NodeBox::toggled, [this, uuid](bool checked) {
            dispatcher_.execute(std::make_shared<command::DisableNode>(uuid, !checked));
        });

        Q_EMIT boxAdded(box);
    }
}

void WidgetController::nodeRemoved(NodeHandlePtr node_handle)
{
    if(designer_) {
        UUID node_uuid = node_handle->getUUID();
        NodeBox* box = getBox(node_uuid);
        box->stop();

        pimpl->box_map_.erase(pimpl->box_map_.find(node_uuid));

        designer_->removeBox(box);

        box->deleteLater();
    }
}

void WidgetController::connectorCreated(ConnectablePtr connector)
{
    // TODO: dirty...
    if(dynamic_cast<Slot*> (connector.get()) || dynamic_cast<Trigger*>(connector.get())) {
        connectorSignalAdded(connector);
    } else {
        connectorMessageAdded(connector);
    }
}

void WidgetController::connectorRemoved(ConnectablePtr connector)
{
    if(designer_) {
        auto it = pimpl->port_map_.find(connector->getUUID());
        if(it != pimpl->port_map_.end()) {
            Port* port = it->second;
            port->deleteLater();
            pimpl->port_map_.erase(it);
        }
    }
}

void WidgetController::connectorMessageAdded(ConnectablePtr connector)
{
    UUID parent_uuid = connector->getUUID().parentUUID();

    Graph* g = graph_->getGraph();
    NodeHandle* node_worker = g->findNodeHandle(parent_uuid);
    if(node_worker) {
        Output* o = dynamic_cast<Output*>(connector.get());
        if(o && node_worker->isParameterOutput(o)) {
            return;
        }

        Input* i = dynamic_cast<Input*>(connector.get());
        if(i && node_worker->isParameterInput(i)) {
            return;
        }

        NodeBox* box = getBox(parent_uuid);
        QBoxLayout* layout = connector->isInput() ? box->getInputLayout() : box->getOutputLayout();

        createPort(connector, box, layout);
    }
}

void WidgetController::connectorSignalAdded(ConnectablePtr connector)
{
    UUID parent_uuid = connector->getUUID().parentUUID();
    NodeBox* box = getBox(parent_uuid);
    QBoxLayout* layout = dynamic_cast<Trigger*>(connector.get()) ? box->getTriggerLayout() : box->getSlotLayout();

    createPort(connector, box, layout);
}

Port* WidgetController::createPort(ConnectableWeakPtr connector, NodeBox* box, QBoxLayout* layout)
{
    apex_assert_hard(QApplication::instance()->thread() == QThread::currentThread());

    if(designer_) {
        Port* port = new Port(&dispatcher_, this, connector);

        if(box) {
            port->setFlipped(box->isFlipped());
            port->setMinimizedSize(box->isMinimizedSize());

            QObject::connect(box, SIGNAL(minimized(bool)), port, SLOT(setMinimizedSize(bool)));
            QObject::connect(box, SIGNAL(flipped(bool)), port, SLOT(setFlipped(bool)));
        }

        insertPort(layout, port);

        return port;
    }

    return nullptr;
}


void WidgetController::insertPort(QLayout* layout, Port* port)
{
    ConnectablePtr adaptee = port->getAdaptee().lock();
    if(!adaptee) {
        return;
    }

    pimpl->port_map_[adaptee->getUUID()] = port;

    layout->addWidget(port);
}



bool WidgetController::isGridLockEnabled() const
{
    return settings_.get<bool>("grid-lock", false);
}

void WidgetController::enableGridLock(bool enabled)
{
    if(!settings_.knows("grid-lock")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("grid-lock", enabled));
    }

    settings_.set("grid-lock", enabled);

    Q_EMIT gridLockEnabled(enabled);
}
/// MOC
#include "../../../include/csapex/view/designer/moc_widget_controller.cpp"
