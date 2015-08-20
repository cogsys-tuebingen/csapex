#ifndef DRAG_IO_H
#define DRAG_IO_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/view/designer/drag_io_handler.h>

/// SYSTEM
#include <QDragEnterEvent>
#include <vector>
#include <boost/type_traits.hpp>

namespace csapex
{

class DragIO
{
public:

public:
    DragIO(PluginLocatorPtr locator, Graph* graph, CommandDispatcher* dispatcher, WidgetControllerPtr widget_ctrl);
    ~DragIO();

    void dragEnterEvent(DesignerView *src, QDragEnterEvent* e);
    void dragMoveEvent(DesignerView *src, QDragMoveEvent* e);
    void dropEvent(DesignerView *src, QDropEvent* e, const QPointF &scene_pos);

public:
    void registerHandler(DragIOHandler::Ptr h);

private:
    void load();

private:
    std::vector<DragIOHandler::Ptr> handler_;

    bool loaded_;
    csapex::PluginLocatorPtr plugin_locator_;
    PluginManager<DragIOHandler>* manager_;

    Graph* graph_;
    CommandDispatcher* dispatcher_;
    WidgetControllerPtr widget_ctrl_;
};

}

#endif // DRAG_IO_H
