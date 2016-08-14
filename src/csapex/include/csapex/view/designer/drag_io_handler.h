#ifndef DRAG_IO_HANDLER_H
#define DRAG_IO_HANDLER_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/command/command_fwd.h>

/// SYSTEM
#include <memory>
#include <QDragEnterEvent>

namespace csapex
{

class GraphView;

class CSAPEX_QT_EXPORT DragIOHandler
{
public:
    typedef std::shared_ptr<DragIOHandler> Ptr;

    virtual ~DragIOHandler();

    virtual bool handleEnter(GraphView* view, CommandDispatcher* dispatcher, QDragEnterEvent* e);
    virtual bool handleMove(GraphView* view, CommandDispatcher* dispatcher, QDragMoveEvent* e);
    virtual bool handleDrop(GraphView* view, CommandDispatcher* dispatcher, QDropEvent* e, const QPointF& scene_pos);

protected:
    DragIOHandler();
};

}

#endif // DRAG_IO_HANDLER_H

