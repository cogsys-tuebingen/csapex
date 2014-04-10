/// HEADER
#include <csapex/view/widget_controller.h>

/// PROJECT
#include <csapex/command/delete_connection.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/manager/box_manager.h>
#include <csapex/view/designer.h>
#include <csapex/view/box.h>
#include <csapex/command/move_box.h>
#include <csapex/model/connectable.h>
#include <csapex/command/move_fulcrum.h>
#include <csapex/view/port.h>
#include <csapex/command/dispatcher.h>

using namespace csapex;

WidgetController::WidgetController(Graph::Ptr graph)
    : graph_(graph), box_selection_(graph, this), connection_selection_(graph, this), designer_(NULL)
{

}

Box* WidgetController::getBox(const UUID &node_id)
{
    return graph_->findNode(node_id)->getBox();
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

void WidgetController::foreachBox(boost::function<void (Box*)> f, boost::function<bool (Box*)> pred)
{
    Q_FOREACH(Node::Ptr n, graph_->nodes_) {
        Box* b = getBox(n->getUUID());
        if(pred(b)) {
            f(b);
        }
    }
}
