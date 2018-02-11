/// HEADER
#include <csapex/view/designer/drag_io_handler.h>

using namespace csapex;

DragIOHandler::DragIOHandler()
{
}

DragIOHandler::~DragIOHandler()
{
}

bool DragIOHandler::handleEnter(GraphView* view, CommandExecutor* dispatcher, QDragEnterEvent* e)
{
    return false;
}
bool DragIOHandler::handleMove(GraphView* view, CommandExecutor* dispatcher, QDragMoveEvent* e)
{
    return false;
}
bool DragIOHandler::handleDrop(GraphView* view, CommandExecutor* dispatcher, QDropEvent* e, const QPointF& scene_pos)
{
    return false;
}
