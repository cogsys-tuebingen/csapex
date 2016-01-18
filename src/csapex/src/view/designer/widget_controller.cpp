/// HEADER
#include <csapex/view/designer/widget_controller.h>

/// PROJECT
#include <csapex/command/delete_connection.h>
#include <csapex/command/disable_node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/core/settings.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/connectable.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/tag.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/trigger.h>
#include <csapex/view/designer/designer.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/node/box.h>
#include <csapex/view/node/default_node_adapter.h>
#include <csapex/view/node/node_adapter_factory.h>
#include <csapex/view/widgets/movable_graphics_proxy_widget.h>
#include <csapex/view/widgets/port.h>

/// SYSTEM
#include <QApplication>
#include <QThread>
#include <QDrag>
#include <QMimeData>
#include <unordered_map>
#include <QPointer>

using namespace csapex;

class WidgetController::Impl
{
public:
    Impl()
    {
    }

    ~Impl()
    {
    }


public:

    QString style_sheet_;
};

WidgetController::WidgetController(Settings& settings, CommandDispatcher& dispatcher, GraphFacade::Ptr graph, NodeFactory* node_factory, NodeAdapterFactory* node_adapter_factory)
    : graph_(graph), dispatcher_(dispatcher), settings_(settings), node_factory_(node_factory), node_adapter_factory_(node_adapter_factory), designer_(nullptr),
      pimpl(new Impl)
{
    if(settings_.knows("grid-lock")) {
        enableGridLock(settings_.get<bool>("grid-lock"));
    }
}

WidgetController::~WidgetController()
{
    for(auto remaining_connections : connections_) {
        remaining_connections.disconnect();
    }
}

GraphFacade::Ptr WidgetController::getGraph()
{
    return graph_;
}

NodeFactory* WidgetController::getNodeFactory()
{
    return node_factory_;
}

void WidgetController::setDesigner(Designer *designer)
{
    designer_ = designer;
}

CommandDispatcher* WidgetController::getCommandDispatcher() const
{
    return &dispatcher_;
}

void WidgetController::setStyleSheet(const QString &str)
{
    pimpl->style_sheet_ = str;
    designer_->overwriteStyleSheet(pimpl->style_sheet_);
}

void WidgetController::startPlacingBox(QWidget *parent, const std::string &type, NodeStatePtr state, const QPoint &offset)
{
    NodeConstructorPtr c = node_factory_->getConstructor(type);
    NodeHandlePtr handle = c->makePrototype();

    QDrag* drag = new QDrag(parent);
    QMimeData* mimeData = new QMimeData;

    mimeData->setData(NodeBox::MIME, type.c_str());
    if(state) {
        QVariant payload = qVariantFromValue((void *) &state);
        mimeData->setProperty("state", payload);
    }
    mimeData->setProperty("ox", offset.x());
    mimeData->setProperty("oy", offset.y());
    drag->setMimeData(mimeData);

    NodeBox::Ptr object(new NodeBox(settings_, handle,
                                    QIcon(QString::fromStdString(c->getIcon()))));
    object->setAdapter(std::make_shared<DefaultNodeAdapter>(handle, object.get()));

    object->setStyleSheet(pimpl->style_sheet_);
    object->construct();
    object->setObjectName(handle->getType().c_str());
    object->setLabel(type);

    drag->setPixmap(object->grab());
    drag->setHotSpot(-offset);
    drag->exec();
}

GraphView* WidgetController::getGraphView()
{
    return designer_->getVisibleGraphView();
}

DesignerScene* WidgetController::getDesignerScene()
{
    return designer_->getVisibleDesignerScene();
}

bool WidgetController::isGridLockEnabled() const
{
    return settings_.get<bool>("grid-lock", false);
}

void WidgetController::enableGridLock(bool enabled)
{
    if(!settings_.knows("grid-lock")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("grid-lock", enabled));
    }

    settings_.set("grid-lock", enabled);

    Q_EMIT gridLockEnabled(enabled);
}
/// MOC
#include "../../../include/csapex/view/designer/moc_widget_controller.cpp"
