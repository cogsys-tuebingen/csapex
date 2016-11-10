#ifndef GRAPH_VIEW_CONTEXT_MENU_H
#define GRAPH_VIEW_CONTEXT_MENU_H

class QPoint;

namespace csapex
{

class GraphView;

class GraphViewContextMenu
{
public:
    GraphViewContextMenu(GraphView& view);

    void showGlobalMenu(const QPoint &global_pos);
    void showSelectionMenu(const QPoint &global_pos);

private:
    GraphView& view_;
};

}

#endif // GRAPH_VIEW_CONTEXT_MENU_H
