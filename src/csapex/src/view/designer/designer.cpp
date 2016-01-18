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
#include <csapex/model/graph_facade.h>
#include <csapex/view/designer/designerio.h>

/// SYSTEM
#include <QScrollBar>
#include <QGLWidget>
#include <QGraphicsView>
#include <QMessageBox>
#include <QGraphicsSceneWheelEvent>
#include <QClipboard>
#include <QMimeData>

using namespace csapex;

Designer::Designer(Settings& settings, GraphFacadePtr main_graph_facade, MinimapWidget *minimap, CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl,
                   DragIO& dragio, QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer),
      drag_io(dragio), minimap_(minimap),
      settings_(settings), root_graph_facade_(main_graph_facade), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl), is_init_(false)
{
    connections_.emplace_back(settings_.saveRequest.connect([this](YAML::Node& node){ saveSettings(node); }));
    connections_.emplace_back(settings_.loadRequest.connect([this](YAML::Node& node){ loadSettings(node); }));

    connections_.emplace_back(settings_.saveDetailRequest.connect([this](Graph* graph, YAML::Node& node){ saveView(graph, node); }));
    connections_.emplace_back(settings_.loadDetailRequest.connect([this](Graph* graph, YAML::Node& node){ loadView(graph, node); }));

    observe(main_graph_facade);
}

Designer::~Designer()
{
    delete ui;
}

void Designer::setup()
{
    ui->setupUi(this);

    addGraph(root_graph_facade_);

//    ui->horizontalLayout->addWidget(minimap_);
    minimap_->setParent(this);
    minimap_->move(10, 10);

    QObject::connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(updateMinimap()));

    updateMinimap();
}

void Designer::observe(GraphFacadePtr graph)
{
    graph->childAdded.connect([this](GraphFacadePtr child){
        addGraph(child);
        observe(child);
    });
    graph->childRemoved.connect([this](GraphFacadePtr child){
        removeGraph(child);
    });
}

void Designer::addGraph(UUID uuid)
{

}

void Designer::addGraph(GraphFacadePtr graph_facade)
{
    // check if it is already displayed
    Graph* graph = graph_facade->getGraph();

    auto pos = graph_tabs_.find(graph);
    if(pos != graph_tabs_.end()) {
        return;
    }


    UUID uuid = graph->getUUID();

    graphs_.push_back(graph_facade);

    DesignerScene* designer_scene = new DesignerScene(graph_facade, dispatcher_, widget_ctrl_, &style);
    GraphView* graph_view = new GraphView(designer_scene, graph_facade, settings_, dispatcher_, widget_ctrl_, drag_io, &style, this);
    graph_views_[graph] = graph_view;
    view_graphs_[graph_view] = graph_facade.get();

    int tab = 0;
    if(graph_tabs_.empty()) {
        // root
        QTabWidget* tabs = ui->tabWidget;
        QIcon icon = tabs->tabIcon(0);
        QString title = tabs->tabText(0);
        tabs->removeTab(0);
        tabs->insertTab(0, graph_view, icon, title);
    } else {
        tab = ui->tabWidget->addTab(graph_view, QString::fromStdString(uuid.getFullName()));
    }

    graph_view->overwriteStyleSheet(styleSheet());


    graph_tabs_[graph] = tab;

    QObject::connect(graph_view, SIGNAL(copyRequest()), this, SLOT(copySelected()));
    QObject::connect(graph_view, SIGNAL(boxAdded(NodeBox*)), this, SLOT(addBox(NodeBox*)));
    QObject::connect(graph_view, SIGNAL(boxRemoved(NodeBox*)), this, SLOT(removeBox(NodeBox*)));

    // main graph tab is not closable
    if(tab == 0) {
        QTabBar *tabBar = ui->tabWidget->findChild<QTabBar *>();
        tabBar->setTabButton(0, QTabBar::RightSide, 0);
        tabBar->setTabButton(0, QTabBar::LeftSide, 0);
    }


    QObject::connect(graph_view, SIGNAL(selectionChanged()), this, SIGNAL(selectionChanged()));

    if(settings_.knows("grid")) {
        enableGrid(settings_.get<bool>("grid"));
    }

    if(settings_.knows("schematics")) {
        enableSchematics(settings_.get<bool>("schematics"));
    }

    if(settings_.knows("display-messages")) {
        designer_scene->displayMessages(settings_.get<bool>("display-messages"));
    }

    if(settings_.knows("display-signals")) {
        designer_scene->displaySignals(settings_.get<bool>("display-signals"));
    }

    if(settings_.knows("debug")) {
        designer_scene->enableDebug(settings_.get<bool>("debug"));
    }
    setFocusPolicy(Qt::NoFocus);
}

void Designer::removeGraph(GraphFacadePtr graph_facade)
{
    std::vector<GraphFacadePtr> graphs_;
    std::map<Graph*, int> graph_tabs_;
    std::map<Graph*, GraphView*> graph_views_;
    std::map<GraphView*, GraphFacade*> view_graphs_;

    for(auto it = graphs_.begin(); it != graphs_.end(); ++it) {
        if(*it == graph_facade) {
            Graph* graph = graph_facade->getGraph();
            GraphView* view = graph_views_[graph];
            graph_tabs_.erase(graph);
            graph_views_.erase(graph);
            view_graphs_.erase(view);
            graphs_.erase(it);
            return;
        }
    }

}

void Designer::updateMinimap()
{
    GraphView* view = getVisibleGraphView();
    minimap_->display(view);
}

std::vector<NodeBox*> Designer::getSelectedBoxes() const
{
    DesignerScene* scene = getVisibleDesignerScene();
    if(!scene) {
        return {};
    }
    return scene->getSelectedBoxes();
}

void Designer::clearSelection()
{
    DesignerScene* scene = getVisibleDesignerScene();
    if(scene) {
        scene->clearSelection();
    }
}

void Designer::selectAll()
{
    GraphView* view = getVisibleGraphView();
    if(!view) {
        return;
    }
    view->selectAll();
}

void Designer::deleteSelected()
{
    GraphView* view = getVisibleGraphView();
    if(!view) {
        return;
    }

    command::Meta::Ptr del(new command::Meta("delete selected"));
    del->add(view->deleteSelected());

    if(del->commands() != 0) {
        dispatcher_->execute(del);
    }
}
void Designer::copySelected()
{
    GraphView* view = getVisibleGraphView();
    if(!view) {
        return;
    }

    GraphIO io(getVisibleGraphFacade()->getGraph(), widget_ctrl_->getNodeFactory());

    std::vector<UUID> nodes;
    for(const NodeBox* box : view->getSelectedBoxes()) {
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
    GraphView* view = getVisibleGraphView();
    if(!view) {
        return;
    }

    const QMimeData* mime = QApplication::clipboard()->mimeData();
    QString data = mime->data("xcsapex/node-list");

    YAML::Node blueprint = YAML::Load(data.toStdString());

    GraphFacade* graph_facade = getVisibleGraphFacade();

    QPointF pos = view->mapToScene(view->mapFromGlobal(QCursor::pos()));

    UUID graph_id = graph_facade->getGraph()->getUUID();
    CommandPtr cmd(new command::PasteGraph(graph_id, blueprint, Point (pos.x(), pos.y())));

    dispatcher_->execute(cmd);
}

void Designer::overwriteStyleSheet(QString &stylesheet)
{
    setStyleSheet(stylesheet);
    for(const auto& facade : graphs_) {
        GraphView* view = graph_views_[facade->getGraph()];
        view->overwriteStyleSheet(stylesheet);
    }
}

void Designer::setView(int /*sx*/, int /*sy*/)
{
    //designer_board->setView(sx, sy);
}

GraphFacade* Designer::getVisibleGraphFacade() const
{
    GraphView* view = getVisibleGraphView();
    if(!view) {
        return nullptr;
    }
    return view_graphs_.at(view);
}

GraphView* Designer::getVisibleGraphView() const
{
    GraphView* current_view = dynamic_cast<GraphView*>(ui->tabWidget->currentWidget());
    if(!current_view) {
        return graph_views_.at(root_graph_facade_->getGraph());
    }
    return current_view;
}

GraphView* Designer::getGraphView(const UUID &uuid) const
{
    for(GraphFacadePtr g : graphs_) {
        Graph* graph = g->getGraph();
        if(graph->getUUID() == uuid) {
            return graph_views_.at(graph);
        }
    }
    return nullptr;
}

DesignerScene* Designer::getVisibleDesignerScene() const
{
    GraphView* view = getVisibleGraphView();
    if(!view) {
        return nullptr;
    }
    return view->designerScene();
}

void Designer::refresh()
{
    DesignerScene* scene = getVisibleDesignerScene();
    if(scene) {
        scene->invalidateSchema();
    }
}

void Designer::reset()
{
    for(const auto& facade : graphs_) {
        GraphView* view = graph_views_[facade->getGraph()];
        view->reset();
    }
}

void Designer::addBox(NodeBox *box)
{
    QObject::connect(box, SIGNAL(helpRequest(NodeBox*)), this, SIGNAL(helpRequest(NodeBox*)));
    minimap_->update();
}

void Designer::removeBox(NodeBox *box)
{
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
    DesignerScene* scene = getVisibleDesignerScene();
    if(!scene) {
        return false;
    }

    return scene->selectedItems().size() > 0;
}

void Designer::enableGrid(bool grid)
{
    if(!settings_.knows("grid")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("grid", grid));
    }

    settings_.set("grid", grid);

    for(const auto& facade : graphs_) {
        GraphView* view = graph_views_[facade->getGraph()];
        view->designerScene()->enableGrid(grid);

        view->setCacheMode(QGraphicsView::CacheNone);
        view->setCacheMode(QGraphicsView::CacheBackground);
    }

    Q_EMIT gridEnabled(grid);

}

void Designer::enableSchematics(bool schema)
{
    if(!settings_.knows("schematics")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("schematics", schema));
    }

    settings_.set("schematics", schema);

    for(const auto& facade : graphs_) {
        GraphView* view = graph_views_[facade->getGraph()];
        view->designerScene()->enableSchema(schema);
    }

    Q_EMIT schematicsEnabled(schema);

}

void Designer::displayGraphComponents(bool display)
{
    if(!settings_.knows("display-graph-components")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-graph-components", display));
    }

    settings_.set("display-graph-components", display);

    for(const auto& facade : graphs_) {
        GraphView* view = graph_views_[facade->getGraph()];
        view->updateBoxInformation();
    }

    Q_EMIT graphComponentsEnabled(display);
}

void Designer::displayThreads(bool display)
{
    if(!settings_.knows("display-threads")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-threads", display));
    }

    settings_.set("display-threads", display);

    for(const auto& facade : graphs_) {
        GraphView* view = graph_views_[facade->getGraph()];
        view->updateBoxInformation();
    }

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

    for(const auto& facade : graphs_) {
        GraphView* view = graph_views_[facade->getGraph()];
        view->designerScene()->displaySignals(display);
    }


    Q_EMIT signalsEnabled(display);
}

void Designer::displayMessageConnections(bool display)
{
    if(!settings_.knows("display-messages")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-messages", display));
    }

    settings_.set("display-messages", display);

    for(const auto& facade : graphs_) {
        GraphView* view = graph_views_[facade->getGraph()];
        view->designerScene()->displayMessages(display);
    }

    Q_EMIT messagesEnabled(display);
}

void Designer::enableDebug(bool debug)
{
    if(!settings_.knows("debug")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("debug", debug));
    }

    settings_.set("debug", debug);

    for(const auto& facade : graphs_) {
        GraphView* view = graph_views_[facade->getGraph()];
        view->designerScene()->enableDebug(debug);
    }

    Q_EMIT debugEnabled(debug);
}

void Designer::saveSettings(YAML::Node& doc)
{
    DesignerIO designerio;
    designerio.saveSettings(doc);
}

void Designer::loadSettings(YAML::Node &doc)
{
    DesignerIO designerio;
    designerio.loadSettings(doc);
}


void Designer::saveView(Graph* graph, YAML::Node &doc)
{
    DesignerIO designerio;
    GraphView *view = graph_views_[graph];
    designerio.saveBoxes(doc, graph, view);
}

void Designer::loadView(Graph* graph, YAML::Node &doc)
{
    DesignerIO designerio;
    GraphView *view = graph_views_[graph];
    designerio.loadBoxes(doc, view);
}
/// MOC
#include "../../../include/csapex/view/designer/moc_designer.cpp"
