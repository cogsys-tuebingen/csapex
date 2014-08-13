/// HEADER
#include <csapex/view/widget_controller.h>

/// PROJECT
#include <csapex/command/delete_connection.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/move_box.h>
#include <csapex/command/move_fulcrum.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/connectable.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/view/box.h>
#include <csapex/view/designer.h>
#include <csapex/view/port.h>
#include <csapex/utility/movable_graphics_proxy_widget.h>
#include <csapex/model/node_worker.h>
#include <csapex/view/default_node_adapter.h>

using namespace csapex;

WidgetController::WidgetController(Graph::Ptr graph, BoxManager* node_factory)
    : graph_(graph), node_factory_(node_factory), designer_(NULL)
{

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

BoxManager* WidgetController::getNodeFactory()
{
    return node_factory_;
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

void WidgetController::nodeAdded(Node::Ptr node)
{
    if(designer_) {
        std::string type = node->getType();

        NodeBox* box;
        QIcon icon = node_factory_->getConstructor(type)->getIcon();
        if(node_factory_->node_adapter_builders_.find(type) != node_factory_->node_adapter_builders_.end()) {
            box = new NodeBox(*node_factory_->settings_, dispatcher_, node, node_factory_->node_adapter_builders_[type]->build(node, this), icon);
        } else {
            box = new NodeBox(*node_factory_->settings_, dispatcher_, node, NodeAdapter::Ptr(new DefaultNodeAdapter(node.get(), this)), icon);
        }
        box->construct();

        box_map_[node->getUUID()] = box;
        proxy_map_[node->getUUID()] = new MovableGraphicsProxyWidget(box, designer_->getDesignerView(), this);

        designer_->addBox(box);

        // add existing connectors
        Q_FOREACH(Input* input, node->getMessageInputs()) {
            connectorAdded(input);
        }
        Q_FOREACH(Output* output, node->getMessageOutputs()) {
            connectorAdded(output);
        }

        // subscribe to coming connectors
        NodeWorker* worker = node->getNodeWorker();
        QObject::connect(worker, SIGNAL(connectorCreated(Connectable*)), this, SLOT(connectorAdded(Connectable*)));
        QObject::connect(worker, SIGNAL(connectorRemoved(Connectable*)), this, SLOT(connectorRemoved(Connectable*)));
    }
}

void WidgetController::nodeRemoved(NodePtr node)
{
    if(designer_) {
        NodeBox* box = getBox(node->getUUID());
        box->stop();

        box_map_.erase(box_map_.find(node->getUUID()));

        designer_->removeBox(box);
    }
}

void WidgetController::connectorAdded(Connectable* connector)
{
    if(designer_) {
        UUID parent_uuid = connector->getUUID().parentUUID();
        NodeBox* box = getBox(parent_uuid);
        Port* port = new Port(dispatcher_, connector, box->isFlipped());


        QObject::connect(box, SIGNAL(minimized(bool)), port, SLOT(setMinimizedSize(bool)));
        QObject::connect(box, SIGNAL(flipped(bool)), port, SLOT(setFlipped(bool)));

        QBoxLayout* layout = connector->isInput() ? box->getInputLayout() : box->getOutputLayout();
        insertPort(layout, port);
    }
}

void WidgetController::insertPort(QLayout* layout, Port* port)
{
    port_map_[port->getAdaptee()->getUUID()] = port;

    layout->addWidget(port);
}


void WidgetController::connectorRemoved(Connectable *connector)
{
    if(designer_) {
        boost::unordered_map<UUID, Port*, UUID::Hasher>::iterator it = port_map_.find(connector->getUUID());
        if(it != port_map_.end()) {
            port_map_.erase(it);
        }
    }
}

void WidgetController::foreachBox(boost::function<void (NodeBox*)> f, boost::function<bool (NodeBox*)> pred)
{
    Q_FOREACH(Node::Ptr n, graph_->nodes_) {
        NodeBox* b = getBox(n->getUUID());
        if(pred(b)) {
            f(b);
        }
    }
}
