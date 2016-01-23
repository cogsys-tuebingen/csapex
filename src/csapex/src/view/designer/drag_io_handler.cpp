/// HEADER
#include <csapex/view/designer/drag_io_handler.h>

using namespace csapex;

DragIOHandler::DragIOHandler()
{
}

DragIOHandler::~DragIOHandler()
{
}

bool DragIOHandler::handleEnter(GraphView* view, CommandDispatcher* dispatcher, QDragEnterEvent* e)
{
    return false;
}
bool DragIOHandler::handleMove(GraphView* view, CommandDispatcher* dispatcher, QDragMoveEvent* e) {
    return false;
}
bool DragIOHandler::handleDrop(GraphView* view, CommandDispatcher* dispatcher, QDropEvent* e, const QPointF& scene_pos)
{
    return false;
}
