/// HEADER
#include <csapex/view/designer/designer.h>

/// COMPONENT
#include <csapex/command/dispatcher.h>
#include <csapex/command/meta.h>
#include <csapex/core/settings.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/node.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/view/node/box.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/designer/designer_scene.h>
#include <csapex/view/widgets/minimap_widget.h>
#include <csapex/core/graphio.h>
#include <csapex/model/graph_facade.h>
#include <csapex/view/designer/designerio.h>
#include "ui_designer.h"

using namespace csapex;

Designer::Designer(Settings& settings, NodeFactory &node_factory, NodeAdapterFactory& node_adapter_factory,
                   GraphFacadePtr main_graph_facade, MinimapWidget *minimap, CommandDispatcher *dispatcher,
                   DragIO& dragio, QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer),
      options_(settings, this), drag_io(dragio), minimap_(minimap),
      settings_(settings), node_factory_(node_factory), node_adapter_factory_(node_adapter_factory),
      root_graph_facade_(main_graph_facade), dispatcher_(dispatcher), is_init_(false)
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

DesignerOptions* Designer::options()
{
    return &options_;
}

void Designer::setup()
{
    ui->setupUi(this);

    addGraph(root_graph_facade_);

//    ui->horizontalLayout->addWidget(minimap_);
    minimap_->setParent(this);
    minimap_->move(10, 10);

    QObject::connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(updateMinimap()));
    QObject::connect(ui->tabWidget, SIGNAL(tabCloseRequested(int)), this, SLOT(closeView(int)));

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

void Designer::showGraph(UUID uuid)
{
    showGraph(graphs_.at(uuid));
}

void Designer::addGraph(GraphFacadePtr graph_facade)
{
    Graph* graph = graph_facade->getGraph();
    UUID uuid = graph->getUUID();

    graphs_[uuid] = graph_facade;

    if(graph_facade == root_graph_facade_) {
        showGraph(graph_facade);
    }
}


void Designer::showGraph(GraphFacadePtr graph_facade)
{
    // check if it is already displayed
    Graph* graph = graph_facade->getGraph();
    auto pos = visible_graphs_.find(graph);
    if(pos != visible_graphs_.end()) {
        // switch to view
        GraphView* view = graph_views_.at(graph_facade->getGraph());
        ui->tabWidget->setCurrentWidget(view);
        return;
    }

    UUID uuid = graph->getUUID();
    DesignerScene* designer_scene = new DesignerScene(graph_facade, dispatcher_, &style);
    GraphView* graph_view = new GraphView(designer_scene, graph_facade,
                                          settings_, node_factory_, node_adapter_factory_,
                                          dispatcher_, drag_io, &style, this);
    graph_views_[graph] = graph_view;
    view_graphs_[graph_view] = graph_facade.get();

    int tab = 0;
    if(visible_graphs_.empty()) {
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


    visible_graphs_.insert(graph);

    ui->tabWidget->setCurrentIndex(tab);

    QObject::connect(graph_view, SIGNAL(boxAdded(NodeBox*)), this, SLOT(addBox(NodeBox*)));
    QObject::connect(graph_view, SIGNAL(boxRemoved(NodeBox*)), this, SLOT(removeBox(NodeBox*)));

    // main graph tab is not closable
    if(tab == 0) {
        QTabBar *tabBar = ui->tabWidget->findChild<QTabBar *>();
        tabBar->setTabButton(0, QTabBar::RightSide, 0);
        tabBar->setTabButton(0, QTabBar::LeftSide, 0);
    }


    QObject::connect(graph_view, SIGNAL(selectionChanged()), this, SIGNAL(selectionChanged()));

    options_.setup(graph_view);

    setFocusPolicy(Qt::NoFocus);
}


void Designer::closeView(int page)
{
    GraphView* view = dynamic_cast<GraphView*>(ui->tabWidget->widget(page));
    if(view) {
        GraphFacade* graph_facade = view_graphs_.at(view);

        Graph* graph = graph_facade->getGraph();

        DesignerIO designerio;
        YAML::Node doc;
        designerio.saveBoxes(doc, graph, graph_views_[graph]);
        states_for_invisible_graphs_[graph->getUUID()] = doc["adapters"];

        ui->tabWidget->removeTab(page);

        visible_graphs_.erase(graph);
        graph_views_.erase(graph);
        view_graphs_.erase(view);
    }
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

bool Designer::hasSelection() const
{
    DesignerScene* scene = getVisibleDesignerScene();
    if(!scene) {
        return false;
    }

    return scene->selectedItems().size() > 0;
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

    view->copySelected();
}

void Designer::paste()
{
    GraphView* view = getVisibleGraphView();
    if(!view) {
        return;
    }

    view->paste();
}

void Designer::overwriteStyleSheet(QString &stylesheet)
{
    setStyleSheet(stylesheet);
    for(const auto& pair : graph_views_) {
        GraphView* view = pair.second;
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
    for(const auto& pair : graph_views_) {
        Graph* graph = pair.first;
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
    for(const auto& pair : graph_views_) {
        GraphView* view = pair.second;
        view->reset();
    }
}

void Designer::addBox(NodeBox *box)
{
    QObject::connect(box, SIGNAL(helpRequest(NodeBox*)), this, SIGNAL(helpRequest(NodeBox*)));
    QObject::connect(box, SIGNAL(showSubGraphRequest(UUID)), this, SLOT(showGraph(UUID)));

    minimap_->update();
}

void Designer::removeBox(NodeBox *box)
{
    minimap_->update();
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

    auto pos = graph_views_.find(graph);
    if(pos != graph_views_.end()) {
        designerio.saveBoxes(doc, graph, pos->second);
        states_for_invisible_graphs_[graph->getUUID()] = doc["adapters"];
    } else {
        doc["adapters"] = states_for_invisible_graphs_[graph->getUUID()];
    }
}

void Designer::loadView(Graph* graph, YAML::Node &doc)
{
    DesignerIO designerio;

    auto pos = graph_views_.find(graph);
    if(pos != graph_views_.end()) {
        designerio.loadBoxes(doc, pos->second);
    }
    states_for_invisible_graphs_[graph->getUUID()] = doc["adapters"];
}
/// MOC
#include "../../../include/csapex/view/designer/moc_designer.cpp"
