#ifndef DRAG_IO_H
#define DRAG_IO_H

/// COMPONENT
#include <csapex/view/designer/drag_io_handler.h>
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/view/view_fwd.h>
#include <csapex/model/model_fwd.h>

/// SYSTEM
#include <QDragEnterEvent>
#include <vector>
#include <boost/type_traits.hpp>

namespace csapex
{

class DragIO
{
public:
    DragIO(PluginLocatorPtr locator, CommandDispatcher* dispatcher);
    ~DragIO();

    void dragEnterEvent(GraphView *src, QDragEnterEvent* e);
    void dragMoveEvent(GraphView *src, QDragMoveEvent* e);
    void dropEvent(GraphView *src, QDropEvent* e, const QPointF &scene_pos);

public:
    void registerHandler(DragIOHandler::Ptr h);

private:
    void createNode(GraphView *src, std::string type, const QPointF &pos,
                    NodeStatePtr state);
    void load();

private:
    std::vector<DragIOHandler::Ptr> handler_;

    bool loaded_;
    csapex::PluginLocatorPtr plugin_locator_;
    PluginManager<DragIOHandler>* manager_;

    CommandDispatcher* dispatcher_;
};

}

#endif // DRAG_IO_H
