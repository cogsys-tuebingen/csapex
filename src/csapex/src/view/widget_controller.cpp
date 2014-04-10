/// HEADER
#include <csapex/view/widget_controller.h>

/// PROJECT
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

void WidgetController::setCommandDispatcher(CommandDispatcher* dispatcher)
{
    dispatcher_ = dispatcher;
}

void WidgetController::nodeAdded(Node::Ptr node)
{
    if(designer_) {
        Box* box = BoxManager::instance().makeBox(node);
        QObject::connect(box, SIGNAL(moveRequest(Box*,QPoint)), this, SLOT(moveSelectedBoxes(Box*, QPoint)));

        designer_->addBox(box);
    }
}

void WidgetController::nodeRemoved(NodePtr node)
{
    if(designer_) {
        Box* box = getBox(node->getUUID());
        designer_->removeBox(box);
    }
}


void WidgetController::moveSelectedBoxes(Box*, const QPoint& delta)
{
    command::Meta::Ptr meta(new command::Meta("Move Selected Boxes"));

    Q_FOREACH(Node::Ptr node, graph_->nodes_) {
        Box* b = getBox(node->getUUID());
        if(b->isSelected()) {
            meta->add(Command::Ptr(new command::MoveBox(b, b->pos())));
        }
    }

    Q_FOREACH(const Connection::Ptr& connection, graph_->connections_) {
        Port* fromp = connection->from()->getPort();
        Port* top = connection->to()->getPort();
        if(!fromp || !top) {
            continue;
        }
        if(fromp->isSelected() && top->isSelected()) {
            int n = connection->getFulcrumCount();
            for(int i = 0; i < n; ++i) {
                const Connection::Fulcrum& f = connection->getFulcrum(i);
                meta->add(Command::Ptr(new command::MoveFulcrum(connection->id(), i, f.pos - delta, f.pos)));
            }
        }
    }

    dispatcher_->execute(meta);
}
