/// HEADER
#include <csapex/view/widget_controller.h>

/// PROJECT
#include <csapex/command/delete_connection.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/move_box.h>
#include <csapex/command/move_fulcrum.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/connectable.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/view/box.h>
#include <csapex/view/designer.h>
#include <csapex/view/port.h>

using namespace csapex;

WidgetController::WidgetController(Graph::Ptr graph)
    : graph_(graph), box_selection_(graph, this), connection_selection_(graph, this), designer_(NULL)
{

}

Box* WidgetController::getBox(const UUID &node_id)
{
    boost::unordered_map<UUID, Box*, UUID::Hasher>::const_iterator pos = box_map_.find(node_id);
    if(pos == box_map_.end()) {
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

void WidgetController::setDesigner(Designer *designer)
{
    designer_ = designer;
}

void WidgetController::setCommandDispatcher(CommandDispatcher* dispatcher)
{
    dispatcher_ = dispatcher;
    box_selection_.setCommandDispatcher(dispatcher);
    connection_selection_.setCommandDispatcher(dispatcher);
}

void WidgetController::nodeAdded(Node::Ptr node)
{
    if(designer_) {
        Box* box = BoxManager::instance().makeBox(node);
        QObject::connect(box, SIGNAL(moveRequest(Box*,QPoint)), &box_selection_, SLOT(moveSelectedBoxes(Box*, QPoint)));

        box_map_[node->getUUID()] = box;

        designer_->addBox(box);

        // add existing connectors
        for(std::size_t i = 0, n = node->countInputs(); i < n; ++i) {
            connectorAdded(node->getInput(i));
        }
        for(std::size_t i = 0, n = node->countOutputs(); i < n; ++i) {
            connectorAdded(node->getOutput(i));
        }

        // subscribe to coming connectors
        QObject::connect(node.get(), SIGNAL(connectorCreated(Connectable*)), this, SLOT(connectorAdded(Connectable*)));
        QObject::connect(node.get(), SIGNAL(connectorRemoved(Connectable*)), this, SLOT(connectorRemoved(Connectable*)));
    }
}

void WidgetController::nodeRemoved(NodePtr node)
{
    if(designer_) {
        Box* box = getBox(node->getUUID());

        box_map_.erase(box_map_.find(node->getUUID()));

        designer_->removeBox(box);
    }
}

void WidgetController::connectorAdded(Connectable* connector)
{
    if(designer_) {
        UUID parent_uuid = connector->getUUID().parentUUID();
        Box* box = getBox(parent_uuid);
        Port* port = new Port(dispatcher_, connector);

        QObject::connect(box, SIGNAL(flipped(bool)), port, SLOT(setFlipped(bool)));
        box->selection.connect(boost::bind(&Port::setSelected, port, _1));

        QBoxLayout* layout = connector->isInput() ? box->getInputLayout() : box->getOutputLayout();

        layout->addWidget(port);

        port_map_[connector->getUUID()] = port;
    }
}

void WidgetController::connectorRemoved(Connectable *connector)
{
    if(designer_) {
        Port* port = getPort(connector->getUUID());

        port_map_.erase(port_map_.find(connector->getUUID()));
    }
}

void WidgetController::foreachBox(boost::function<void (Box*)> f, boost::function<bool (Box*)> pred)
{
    Q_FOREACH(Node::Ptr n, graph_->nodes_) {
        Box* b = getBox(n->getUUID());
        if(pred(b)) {
            f(b);
        }
    }
}
