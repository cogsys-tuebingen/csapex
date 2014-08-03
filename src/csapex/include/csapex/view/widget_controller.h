#ifndef WIDGET_CONTROLLER_H
#define WIDGET_CONTROLLER_H

/// PROJECT
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QObject>
#include <QPoint>
#include <boost/function.hpp>
#include <boost/unordered_map.hpp>
#include <QLayout>

namespace csapex
{

class WidgetController : public QObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<WidgetController> Ptr;

public:
    WidgetController(GraphPtr graph);

    NodeBox* getBox(const UUID& node_id);

    MovableGraphicsProxyWidget* getProxy(const UUID& node_id);

    Port* getPort(const UUID& connector_id);
    Port* getPort(const Connectable* connector_id);

    GraphPtr getGraph();

    void setDesigner(Designer* designer);

    CommandDispatcher* getCommandDispatcher() const;
    void setCommandDispatcher(CommandDispatcher *dispatcher);

    void foreachBox(boost::function<void (NodeBox*)> f, boost::function<bool (NodeBox*)> pred);

public Q_SLOTS:
    void nodeAdded(NodePtr node);
    void nodeRemoved(NodePtr node);

    void connectorAdded(Connectable *connector);
    void connectorRemoved(Connectable *connector);

    void insertPort(QLayout* layout, Port* port);

private:
    GraphPtr graph_;
    CommandDispatcher* dispatcher_;

private:
    Designer* designer_;
    boost::unordered_map<UUID, NodeBox*, UUID::Hasher> box_map_;
    boost::unordered_map<UUID, MovableGraphicsProxyWidget*, UUID::Hasher> proxy_map_;
    boost::unordered_map<UUID, Port*, UUID::Hasher> port_map_;
};

}

#endif // WIDGET_CONTROLLER_H
