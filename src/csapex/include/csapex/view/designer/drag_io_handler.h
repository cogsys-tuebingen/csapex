#ifndef DRAG_IO_HANDLER_H
#define DRAG_IO_HANDLER_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <memory>
#include <QDragEnterEvent>

namespace csapex
{

class DragIOHandler
{
public:
    typedef std::shared_ptr<DragIOHandler> Ptr;

    virtual ~DragIOHandler();

    virtual bool handleEnter(CommandDispatcher* dispatcher, QWidget *src, QDragEnterEvent* e);
    virtual bool handleMove(CommandDispatcher* dispatcher, QWidget *src, QDragMoveEvent* e);
    virtual bool handleDrop(CommandDispatcher* dispatcher, QWidget *src, QDropEvent* e, const QPointF& scene_pos);

protected:
    DragIOHandler();
};

}

#endif // DRAG_IO_HANDLER_H

