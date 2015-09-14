#ifndef WIDGET_CONTROLLER_H
#define WIDGET_CONTROLLER_H

/// PROJECT
#include <csapex/utility/uuid.h>
#include <csapex/core/core_fwd.h>
#include <csapex/command/command_fwd.h>
#include <csapex/view/view_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/factory/factory_fwd.h>
#include <csapex/param/param_fwd.h>

/// SYSTEM
#include <memory>
#include <QObject>
#include <QPoint>
#include <QLayout>
#include <QGraphicsView>
#include <boost/signals2/connection.hpp>

class QTreeWidget;
class QMenu;
class QAbstractItemModel;

namespace csapex
{

class WidgetController : public QObject
{
    Q_OBJECT

public:
    typedef std::shared_ptr<WidgetController> Ptr;

public:
    WidgetController(Settings& settings, CommandDispatcher& dispatcher, GraphWorkerPtr graph, NodeFactory* node_factory, NodeAdapterFactory* node_adapter_factory);
    ~WidgetController();

    void startPlacingBox(QWidget *parent, const std::string& type, NodeStatePtr state, const QPoint &offset = QPoint(0,0));

    DesignerView* getDesignerView();
    DesignerScene* getDesignerScene();

    NodeBox* getBox(const UUID& node_id);

    MovableGraphicsProxyWidget* getProxy(const UUID& node_id);


    //** FACTORY ??? **//
    Port *createPort(ConnectableWeakPtr connector, NodeBox *box, QBoxLayout *layout);

    Port* getPort(const UUID& connector_id);
    Port* getPort(const Connectable* connector_id);

    GraphWorkerPtr getGraph();
    NodeFactory* getNodeFactory();

    void hideTooltipView();
    QGraphicsView* getTooltipView(const std::string& title);

    void setDesigner(Designer* designer);

    CommandDispatcher* getCommandDispatcher() const;

    void setStyleSheet(const QString &str);


    // TODO: extract into something like NodeTypeListGenerator
    void insertAvailableNodeTypes(QMenu* menu);
    void insertAvailableNodeTypes(QTreeWidget *tree);
    QAbstractItemModel *listAvailableNodeTypes();

    bool isGridLockEnabled() const;

Q_SIGNALS:
    void gridLockEnabled(bool);

    void boxAdded(NodeBox* box);

    void triggerConnectorCreated(ConnectablePtr connector);
    void triggerConnectorRemoved(ConnectablePtr connector);

public Q_SLOTS:
    void nodeAdded(NodeWorkerPtr node_worker);
    void nodeRemoved(NodeWorkerPtr node_worker);

    void connectorCreated(ConnectablePtr connector);
    void connectorRemoved(ConnectablePtr connector);

    void enableGridLock(bool enabled);

private:
    void insertPort(QLayout* layout, Port* port);

    void connectorSignalAdded(ConnectablePtr connector);

    void connectorMessageAdded(ConnectablePtr connector);

private:

    GraphWorkerPtr graph_;
    CommandDispatcher& dispatcher_;
    Settings& settings_;
    NodeFactory* node_factory_;
    NodeAdapterFactory* node_adapter_factory_;
    Designer* designer_;

    std::vector<boost::signals2::connection> connections_;

    class Impl;
    std::unique_ptr<Impl> pimpl;
};

}

#endif // WIDGET_CONTROLLER_H
