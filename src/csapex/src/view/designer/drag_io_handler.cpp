/// HEADER
#include <csapex/view/designer/drag_io_handler.h>

using namespace csapex;

DragIOHandler::DragIOHandler()
{
}

DragIOHandler::~DragIOHandler()
{
}

bool DragIOHandler::handleEnter(CommandDispatcher* dispatcher, QWidget *src, QDragEnterEvent* e)
{
    return false;
}
bool DragIOHandler::handleMove(CommandDispatcher* dispatcher, QWidget *src, QDragMoveEvent* e) {
    return false;
}
bool DragIOHandler::handleDrop(CommandDispatcher* dispatcher, QWidget *src, QDropEvent* e, const QPointF& scene_pos)
{
    return false;
}
