/// HEADER
#include <csapex/view/designer/designer.h>

/// COMPONENT
#include <csapex/command/dispatcher.h>
#include <csapex/command/meta.h>
#include <csapex/command/paste_graph.h>
#include <csapex/core/settings.h>
#include <csapex/factory/node_factory.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/node.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/view/node/box.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/designer/widget_controller.h>
#include "ui_designer.h"
#include <csapex/param/parameter_factory.h>
#include <csapex/view/designer/designer_scene.h>
#include <csapex/view/widgets/minimap_widget.h>
#include <csapex/core/graphio.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <QScrollBar>
#include <QGLWidget>
#include <QGraphicsView>
#include <QMessageBox>
#include <QGraphicsSceneWheelEvent>
#include <QClipboard>
#include <QMimeData>

using namespace csapex;

Designer::Designer(Settings& settings, Graph::Ptr graph, CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl,
                   GraphView* view, DesignerScene* scene, MinimapWidget* minimap,
                   QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer),
      main_graph_view_(view), designer_scene_(scene), minimap_(minimap),
      settings_(settings), graph_(graph), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl), is_init_(false)
{
    QObject::connect(view, SIGNAL(copyRequest()), this, SLOT(copySelected()));
}

Designer::~Designer()
{
    delete ui;
}

void Designer::setup()
{
    ui->setupUi(this);

    QLayout* main_layout = new QVBoxLayout;
    main_layout->addWidget(main_graph_view_);
    ui->tab_main_graph->setLayout(main_layout);


    // main graph tab is not closable
    QTabBar *tabBar = ui->tabWidget->findChild<QTabBar *>();
    tabBar->setTabButton(0, QTabBar::RightSide, 0);
    tabBar->setTabButton(0, QTabBar::LeftSide, 0);


    QObject::connect(main_graph_view_, SIGNAL(selectionChanged()), this, SIGNAL(selectionChanged()));

    if(settings_.knows("grid")) {
        enableGrid(settings_.get<bool>("grid"));
    }

    if(settings_.knows("schematics")) {
        enableSchematics(settings_.get<bool>("schematics"));
    }

    if(settings_.knows("display-messages")) {
        designer_scene_->displayMessages(settings_.get<bool>("display-messages"));
    }

    if(settings_.knows("display-signals")) {
        designer_scene_->displaySignals(settings_.get<bool>("display-signals"));
    }

    if(settings_.knows("debug")) {
        designer_scene_->enableDebug(settings_.get<bool>("debug"));
    }
    setFocusPolicy(Qt::NoFocus);

//    ui->horizontalLayout->addWidget(minimap_);
    minimap_->setParent(main_graph_view_);
    minimap_->move(10, 10);
}

std::vector<NodeBox*> Designer::getSelectedBoxes() const
{
    return designer_scene_->getSelectedBoxes();
}

void Designer::clearSelection()
{
    main_graph_view_->scene()->clearSelection();
}

void Designer::selectAll()
{
    main_graph_view_->selectAll();
}

void Designer::deleteSelected()
{
    command::Meta::Ptr del(new command::Meta("delete selected"));

    del->add(main_graph_view_->deleteSelected());

    if(del->commands() != 0) {
        dispatcher_->execute(del);
    }
}
void Designer::copySelected()
{
    GraphIO io(graph_.get(), widget_ctrl_->getNodeFactory());

    std::vector<UUID> nodes;
    for(const NodeBox* box : main_graph_view_->getSelectedBoxes()) {
        nodes.emplace_back(box->getNodeHandle()->getUUID());
    }

    YAML::Node yaml;
    io.saveSelectedGraph(yaml, nodes);

    QMimeData* mime = new QMimeData;
    std::stringstream yaml_txt;
    yaml_txt << yaml;

    auto data = QString::fromStdString(yaml_txt.str()).toUtf8();
    mime->setData("text/plain", data);
    mime->setData("text/yaml", data);
    mime->setData("xcsapex/node-list", data);

    QApplication::clipboard()->setMimeData(mime, QClipboard::Clipboard);
}

void Designer::paste()
{
    const QMimeData* mime = QApplication::clipboard()->mimeData();
    QString data = mime->data("xcsapex/node-list");

    YAML::Node blueprint = YAML::Load(data.toStdString());

    QPointF pos = main_graph_view_->mapToScene(main_graph_view_->mapFromGlobal(QCursor::pos()));

    UUID graph_id = graph_->getUUID();
    CommandPtr cmd(new command::PasteGraph(graph_id, blueprint, Point (pos.x(), pos.y())));

    dispatcher_->execute(cmd);
}

void Designer::overwriteStyleSheet(QString &stylesheet)
{
    main_graph_view_->overwriteStyleSheet(stylesheet);
}

void Designer::setView(int /*sx*/, int /*sy*/)
{
    //designer_board->setView(sx, sy);
}

GraphView* Designer::getGraphView()
{
    return main_graph_view_;
}

void Designer::refresh()
{
    designer_scene_->invalidateSchema();
}

void Designer::reset()
{
    main_graph_view_->reset();
}

void Designer::addBox(NodeBox *box)
{
    QObject::connect(box, SIGNAL(helpRequest(NodeBox*)), this, SIGNAL(helpRequest(NodeBox*)));

    main_graph_view_->addBoxEvent(box);

    minimap_->update();
}

void Designer::removeBox(NodeBox *box)
{
    main_graph_view_->removeBoxEvent(box);

    minimap_->update();
}

bool Designer::isGridEnabled() const
{
    return settings_.get<bool>("grid", false);
}
bool Designer::isSchematicsEnabled() const
{
    return settings_.get<bool>("schematics", false);
}

bool Designer::isGraphComponentsEnabled() const
{
    return settings_.get<bool>("display-graph-components", false);
}
bool Designer::isThreadsEnabled() const
{
    return settings_.get<bool>("display-threads", false);
}

bool Designer::isMinimapEnabled() const
{
    return settings_.get<bool>("display-minimap", false);
}

bool Designer::areMessageConnectionsVisibile() const
{
    return settings_.get<bool>("display-messages", true);
}

bool Designer::areSignalConnectionsVisible() const
{
    return settings_.get<bool>("display-signals", true);
}
bool Designer::isDebug() const
{
    return settings_.get<bool>("debug", false);
}

bool Designer::hasSelection() const
{
    return designer_scene_->selectedItems().size() > 0;
}

void Designer::enableGrid(bool grid)
{
    if(!settings_.knows("grid")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("grid", grid));
    }

    settings_.set("grid", grid);

    designer_scene_->enableGrid(grid);

    main_graph_view_->setCacheMode(QGraphicsView::CacheNone);
    main_graph_view_->setCacheMode(QGraphicsView::CacheBackground);

    Q_EMIT gridEnabled(grid);

}

void Designer::enableSchematics(bool schema)
{
    if(!settings_.knows("schematics")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("schematics", schema));
    }

    settings_.set("schematics", schema);

    designer_scene_->enableSchema(schema);

    Q_EMIT schematicsEnabled(schema);

}

void Designer::displayGraphComponents(bool display)
{
    if(!settings_.knows("display-graph-components")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-graph-components", display));
    }

    settings_.set("display-graph-components", display);

    main_graph_view_->updateBoxInformation();

    Q_EMIT graphComponentsEnabled(display);
}

void Designer::displayThreads(bool display)
{
    if(!settings_.knows("display-threads")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-threads", display));
    }

    settings_.set("display-threads", display);

    main_graph_view_->updateBoxInformation();

    Q_EMIT threadsEnabled(display);
}


void Designer::displayMinimap(bool display)
{
    if(!settings_.knows("display-minimap")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-minimap", display));
    }

    settings_.set("display-minimap", display);

    minimap_->setVisible(display);

    Q_EMIT minimapEnabled(display);
}

void Designer::displaySignalConnections(bool display)
{
    if(!settings_.knows("display-signals")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-signals", display));
    }

    settings_.set("display-signals", display);

    designer_scene_->displaySignals(display);


    Q_EMIT signalsEnabled(display);
}

void Designer::displayMessageConnections(bool display)
{
    if(!settings_.knows("display-messages")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-messages", display));
    }

    settings_.set("display-messages", display);

    designer_scene_->displayMessages(display);

    Q_EMIT messagesEnabled(display);
}

void Designer::enableDebug(bool debug)
{
    if(!settings_.knows("debug")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("debug", debug));
    }

    settings_.set("debug", debug);

    designer_scene_->enableDebug(debug);

    Q_EMIT debugEnabled(debug);
}
/// MOC
#include "../../../include/csapex/view/designer/moc_designer.cpp"
