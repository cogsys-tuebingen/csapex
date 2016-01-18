/// HEADER
#include <csapex/command/move_box.h>

/// COMPONENT
#include <csapex/view/node/box.h>
#include <csapex/model/graph.h>
#include <csapex/view/designer/widget_controller.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/widgets/movable_graphics_proxy_widget.h>

/// SYSTEM
#include <QPoint>
#include <sstream>

using namespace csapex::command;

MoveBox::MoveBox(const UUID& node_uuid, const UUID& graph_uuid, Point from, Point to, Designer *view)
    : view_(view), from(from), to(to), graph_uuid(graph_uuid), box_uuid(node_uuid)
{
}

std::string MoveBox::getType() const
{
    return "MoveBox";
}

std::string MoveBox::getDescription() const
{
    std::stringstream ss;
    ss << "moved box " << box_uuid << " from (" << from.x << ", " << from.y << ") to";
    ss << "(" << to.x << ", " << to.y << ")";
    return ss.str();
}


bool MoveBox::doExecute()
{
    MovableGraphicsProxyWidget* box = view_->getGraphView(graph_uuid)->getProxy(box_uuid);
    box->getBox()->triggerPlaced();
    return true;
}

bool MoveBox::doUndo()
{
    MovableGraphicsProxyWidget* box = view_->getGraphView(graph_uuid)->getProxy(box_uuid);
    box->setPos(QPoint(from.x, from.y));
    box->getBox()->triggerPlaced();
    return true;
}

bool MoveBox::doRedo()
{
    MovableGraphicsProxyWidget* box = view_->getGraphView(graph_uuid)->getProxy(box_uuid);
    box->setPos(QPoint(to.x, to.y));
    box->getBox()->triggerPlaced();
    return true;
}
