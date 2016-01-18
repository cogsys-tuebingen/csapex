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
#include <csapex/utility/slim_signal.hpp>

/// SYSTEM
#include <memory>
#include <QObject>
#include <QPoint>
#include <QLayout>
#include <QGraphicsView>

namespace csapex
{

class WidgetController : public QObject
{
    Q_OBJECT

    // TODO: remove
    friend class GraphView;

public:
    typedef std::shared_ptr<WidgetController> Ptr;

public:
    WidgetController(Settings& settings, CommandDispatcher& dispatcher, GraphFacadePtr graph, NodeFactory* node_factory, NodeAdapterFactory* node_adapter_factory);
    ~WidgetController();

    void startPlacingBox(QWidget *parent, const std::string& type, NodeStatePtr state, const QPoint &offset = QPoint(0,0));

    NodeFactory* getNodeFactory();

    CommandDispatcher* getCommandDispatcher() const;

    void setStyleSheet(const QString &str);

    bool isGridLockEnabled() const;

Q_SIGNALS:
    void gridLockEnabled(bool);

    void boxAdded(NodeBox* box);

public Q_SLOTS:
    void enableGridLock(bool enabled);

private:

    CommandDispatcher& dispatcher_;
    Settings& settings_;
    NodeFactory* node_factory_;
    NodeAdapterFactory* node_adapter_factory_;

    std::vector<csapex::slim_signal::Connection> connections_;

    class Impl;
    std::unique_ptr<Impl> pimpl;
};

}

#endif // WIDGET_CONTROLLER_H
