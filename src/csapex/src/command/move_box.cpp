/// HEADER
#include <csapex/command/move_box.h>

/// COMPONENT
#include <csapex/model/graph.h>
//#include <csapex/view/node/box.h>
//#include <csapex/view/designer/graph_view.h>
//#include <csapex/view/widgets/movable_graphics_proxy_widget.h>
//#include <csapex/view/designer/designer.h>

/// SYSTEM
//#include <QPoint>
#include <sstream>

using namespace csapex;
using namespace csapex::command;

MoveBox::MoveBox(const AUUID& graph_uuid, const UUID& node_uuid, Point from, Point to)
    : Command(graph_uuid), from(from), to(to), box_uuid(node_uuid)
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
   /* Designer* designer = getDesigner();
    if(!designer) {
        return false;
    }
    MovableGraphicsProxyWidget* box = designer->getGraphView(graph_uuid)->getProxy(box_uuid);
    if(!box) {
        return false;
    }
    box->getBox()->triggerPlaced();*/
    return true;
}

bool MoveBox::doUndo()
{
   /* Designer* designer = getDesigner();
    if(!designer) {
        return false;
    }
    MovableGraphicsProxyWidget* box = designer->getGraphView(graph_uuid)->getProxy(box_uuid);
    if(!box) {
        return false;
    }
    box->setPos(QPoint(from.x, from.y));
    box->getBox()->triggerPlaced();*/
    return true;
}

bool MoveBox::doRedo()
{
    /*Designer* designer = getDesigner();
    if(!designer) {
        return false;
    }
    MovableGraphicsProxyWidget* box = designer->getGraphView(graph_uuid)->getProxy(box_uuid);
    if(!box) {
        return false;
    }
    box->setPos(QPoint(to.x, to.y));
    box->getBox()->triggerPlaced();*/
    return true;
}
