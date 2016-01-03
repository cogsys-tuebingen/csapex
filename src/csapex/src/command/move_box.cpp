/// HEADER
#include <csapex/command/move_box.h>

/// COMPONENT
//#include <csapex/model/node.h>
#include <csapex/view/node/box.h>
#include <csapex/model/graph.h>
#include <csapex/view/designer/widget_controller.h>
#include <csapex/view/widgets/movable_graphics_proxy_widget.h>

/// SYSTEM
#include <QPoint>
#include <sstream>

using namespace csapex::command;

MoveBox::MoveBox(const UUID& node_uuid, Point from, Point to, WidgetController &widget_controller)
    : widget_controller_(widget_controller), from(from), to(to), uuid(node_uuid)
{
}

std::string MoveBox::getType() const
{
    return "MoveBox";
}

std::string MoveBox::getDescription() const
{
    std::stringstream ss;
    ss << "moved box " << uuid << " from (" << from.x << ", " << from.y << ") to";
    ss << "(" << to.x << ", " << to.y << ")";
    return ss.str();
}


bool MoveBox::doExecute()
{
    MovableGraphicsProxyWidget* box = widget_controller_.getProxy(uuid);
    box->getBox()->triggerPlaced();
    return true;
}

bool MoveBox::doUndo()
{
    MovableGraphicsProxyWidget* box = widget_controller_.getProxy(uuid);
    box->setPos(QPoint(from.x, from.y));
    box->getBox()->triggerPlaced();
    return true;
}

bool MoveBox::doRedo()
{
    MovableGraphicsProxyWidget* box = widget_controller_.getProxy(uuid);
    box->setPos(QPoint(to.x, to.y));
    box->getBox()->triggerPlaced();
    return true;
}
