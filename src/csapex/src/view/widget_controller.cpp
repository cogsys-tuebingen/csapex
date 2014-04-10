/// HEADER
#include <csapex/view/widget_controller.h>

/// PROJECT
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/manager/box_manager.h>
#include <csapex/view/designer.h>

using namespace csapex;

WidgetController::WidgetController(Graph::Ptr graph)
    : graph_(graph), designer_(NULL)
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

void WidgetController::nodeAdded(Node::Ptr node)
{
    if(designer_) {
        designer_->addBox(BoxManager::instance().makeBox(node));
    }
}

void WidgetController::nodeRemoved(NodePtr node)
{
    if(designer_) {
        designer_->removeBox(getBox(node->getUUID()));
    }
}
