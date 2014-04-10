#ifndef WIDGET_CONTROLLER_H
#define WIDGET_CONTROLLER_H

/// PROJECT
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QObject>
#include <QPoint>
#include <QMenu>
#include <boost/function.hpp>

namespace csapex
{

class SelectionManager  : public QObject
{
    Q_OBJECT

public:
    SelectionManager(GraphPtr graph, WidgetController* widget_ctrl)
        : graph_(graph), widget_ctrl_(widget_ctrl)
    {
    }

    virtual ~SelectionManager()
    {
    }

    void setCommandDispatcher(CommandDispatcher *dispatcher)
    {
        dispatcher_ = dispatcher;
    }

Q_SIGNALS:
    void selectionChanged();

protected:
    GraphPtr graph_;
    WidgetController* widget_ctrl_;
    CommandDispatcher *dispatcher_;
};

class BoxSelectionManager : public SelectionManager
{
    Q_OBJECT

public:
    BoxSelectionManager(GraphPtr graph, WidgetController* widget_ctrl);

    void handleNodeSelection(Node* node, bool add);
    CommandPtr deleteSelectedNodesCmd();

    void selectNode(Node* node, bool add = false);
    void deselectNodes();

    int countSelectedNodes();

    void fillContextMenuForSelection(QMenu* menu, std::map<QAction *, boost::function<void()> > &handler);


public Q_SLOTS:
    void clearSelection();
    void selectAll();

    void toggleBoxSelection(Box* box);
    void boxMoved(Box* box, int dx, int dy);

    void moveSelectedBoxes(Box* origin, const QPoint& delta);
};

class ConnectionSelectionManager : public SelectionManager
{
    Q_OBJECT

public:
    ConnectionSelectionManager(GraphPtr graph, WidgetController* widget_ctrl);

    CommandPtr deleteSelectedConnectionsCmd();

public:
    bool handleConnectionSelection(int id, bool add);
    int countSelectedConnections();

private:
    void deselectConnections();
    void deselectConnectionById(int id);

    void selectConnectionById(int id, bool add = false);
    bool isConnectionWithIdSelected(int id);
};


class WidgetController : public QObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<WidgetController> Ptr;

public:
    WidgetController(GraphPtr graph);

    Box* getBox(const UUID& node_id);

    GraphPtr getGraph();

    void setDesigner(Designer* designer);
    void setCommandDispatcher(CommandDispatcher *dispatcher);

public Q_SLOTS:
    void nodeAdded(NodePtr node);
    void nodeRemoved(NodePtr node);

private:
    GraphPtr graph_;
    CommandDispatcher* dispatcher_;

public:
    BoxSelectionManager box_selection_;
    ConnectionSelectionManager connection_selection_;

private:
    Designer* designer_;
};

}

#endif // WIDGET_CONTROLLER_H
