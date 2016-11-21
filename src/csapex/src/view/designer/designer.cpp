/// HEADER
#include <csapex/view/designer/designer.h>

/// COMPONENT
#include <csapex/command/dispatcher.h>
#include <csapex/command/meta.h>
#include <csapex/command/rename_node.h>
#include <csapex/core/settings.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/node.h>
#include <csapex/model/node_state.h>
#include <csapex/core/csapex_core.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/view/node/box.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/designer/designer_scene.h>
#include <csapex/view/widgets/minimap_widget.h>
#include <csapex/core/graphio.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/view/designer/designerio.h>
#include "ui_designer.h"
#include <csapex/view/widgets/search_dialog.h>
#include <csapex/view/widgets/notification_widget.h>

/// SYSTEM
#include <QTabWidget>
#include <QInputDialog>
#include <QResizeEvent>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>

using namespace csapex;

Designer::Designer(CsApexViewCore& view_core, QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer),
      options_(view_core.getSettings(), this),
      minimap_(new MinimapWidget),
      core_(view_core.getCore()), view_core_(view_core), is_init_(false),
      notification_animation_(nullptr)
{
    view_core.getCommandDispatcher().setDesigner(this);

    observe(core_.getSettings().save_request, [this](YAML::Node& node){ saveSettings(node); });
    observe(core_.getSettings().load_request, [this](YAML::Node& node){ loadSettings(node); });

    observe(core_.getSettings().save_detail_request, [this](SubgraphNode* graph, YAML::Node& node){ saveView(graph, node); });
    observe(core_.getSettings().load_detail_request, [this](SubgraphNode* graph, YAML::Node& node){ loadView(graph, node); });

    observeGraph(core_.getRoot());
}

Designer::~Designer()
{
    delete ui;
    delete notification_animation_;
}

DesignerOptions* Designer::options()
{
    return &options_;
}

MinimapWidget* Designer::getMinimap()
{
    return minimap_;
}

void Designer::resizeEvent(QResizeEvent *re)
{
    QWidget::resizeEvent(re);
}

void Designer::setup()
{
    ui->setupUi(this);

    addGraph(core_.getRoot());

    //    ui->horizontalLayout->addWidget(minimap_);
    minimap_->setParent(this);
    minimap_->move(10, 10);

    QObject::connect(ui->tabWidget, &QTabWidget::currentChanged,
                     [this](int) {
        updateMinimap();
    });
    QObject::connect(ui->tabWidget, &QTabWidget::tabCloseRequested,
                     [this](int tab) {
        closeView(tab);
    });
    QObject::connect(ui->tabWidget, &QTabWidget::tabBarDoubleClicked,
                     [this](int tab) {
        bool ok = false;
        GraphView* view = dynamic_cast<GraphView*>(ui->tabWidget->widget(tab));
        if(view) {
            GraphFacade* graph = view->getGraphFacade();
            GraphFacade* parent = graph->getParent();
            if(parent) {
                NodeHandle* node = graph->getNodeHandle();
                NodeStatePtr state = node->getNodeState();
                QString old_name = QString::fromStdString(state->getLabel());
                QString text = QInputDialog::getText(this, "Graph Label", "Enter new name",
                                                     QLineEdit::Normal, old_name, &ok);
                if(ok) {
                    if(old_name != text && !text.isEmpty()) {
                        command::RenameNode::Ptr cmd(new command::RenameNode(parent->getAbsoluteUUID(),
                                                                             node->getUUID(),
                                                                             text.toStdString()));
                        view_core_.execute(cmd);
                    }
                }
            }
        }
    });

    updateMinimap();
}

void Designer::observeGraph(GraphFacadePtr graph)
{
    graph_connections_[graph->getSubgraphNode()].emplace_back(
                graph->child_added.connect([this](GraphFacadePtr child){
                    addGraph(child);
                    observeGraph(child);
                }));
    graph_connections_[graph->getSubgraphNode()].emplace_back(
                graph->child_removed.connect([this](GraphFacadePtr child){
                    removeGraph(child.get());
                }));
}

void Designer::showGraph(UUID uuid)
{
    showGraph(graphs_.at(uuid));
}

void Designer::showNodeDialog()
{
    if(GraphView* current_view = dynamic_cast<GraphView*>(ui->tabWidget->currentWidget())) {
        current_view->showNodeInsertDialog();
    }
}

void Designer::showNodeSearchDialog()
{
    if(GraphView* current_view = dynamic_cast<GraphView*>(ui->tabWidget->currentWidget())) {
        SearchDialog diag(current_view->getGraphFacade()->getGraph(), core_.getNodeFactory(),
                          "Please enter the UUID, the label or the type of the node");

        int r = diag.exec();

        if(r) {
            Q_EMIT focusOnNode(diag.getAUUID());
        }
    }
}


void Designer::addGraph(GraphFacadePtr graph_facade)
{
    UUID uuid = graph_facade->getAbsoluteUUID();

    graphs_[uuid] = graph_facade;

    if(graph_facade == core_.getRoot()) {
        showGraph(graph_facade);
    }
}

namespace {
QString generateTitle(GraphFacade* graph_facade)
{
    QString title;
    for(GraphFacade* parent = graph_facade; parent != nullptr; parent = parent->getParent()) {
        NodeHandle* nh = parent->getNodeHandle();
        if(!nh) break;

        QString label = QString::fromStdString(nh->getNodeState()->getLabel());
        if(!title.isEmpty()) {
            title = label + " / " + title;

        } else {
            title = label;
        }
    }

    if(title.isEmpty()) {
        return "Main";
    } else {
        return title;
    }
}
}


void Designer::showGraph(GraphFacadePtr graph_facade)
{
    // check if it is already displayed
    SubgraphNode* graph = graph_facade->getSubgraphNode();
    auto pos = visible_graphs_.find(graph);
    if(pos != visible_graphs_.end()) {
        // switch to view
        GraphView* view = graph_views_.at(graph_facade->getSubgraphNode());
        ui->tabWidget->setCurrentWidget(view);
        return;
    }

    GraphView* graph_view = new GraphView(graph_facade, view_core_, this);
    graph_view->useProfiler(profiler_);
    graph_views_[graph] = graph_view;
    view_graphs_[graph_view] = graph_facade.get();
    auuid_views_[graph_facade->getAbsoluteUUID()] = graph_view;

    int tab = 0;
    if(visible_graphs_.empty()) {
        // root
        QTabWidget* tabs = ui->tabWidget;
        QIcon icon(":/step_next.png");
        tabs->removeTab(0);
        tabs->insertTab(0, graph_view, icon, generateTitle(graph_facade.get()));
    } else {
        tab = ui->tabWidget->addTab(graph_view, generateTitle(graph_facade.get()));

        view_connections_[graph_view].emplace_back(graph_facade->getNodeHandle()->getNodeState()->label_changed->connect([this]() {
            for(int i = 0; i < ui->tabWidget->count(); ++i) {
                GraphView* view = dynamic_cast<GraphView*>(ui->tabWidget->widget(i));
                if(view) {
                    ui->tabWidget->setTabText(i, generateTitle(view->getGraphFacade()));
                }
            }
        }));
    }

    graph_view->overwriteStyleSheet(styleSheet());


    visible_graphs_.insert(graph);

    ui->tabWidget->setCurrentIndex(tab);

    QObject::connect(graph_view, &GraphView::boxAdded, this, &Designer::addBox);
    QObject::connect(graph_view, &GraphView::boxRemoved, this, &Designer::removeBox);

    for(const auto& nh : graph->getAllNodeHandles()) {
        NodeBox* box = graph_view->getBox(nh->getUUID());
        addBox(box);
    }

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

        SubgraphNode* graph = graph_facade->getSubgraphNode();

        DesignerIO designerio;
        YAML::Node doc;
        designerio.saveBoxes(doc, graph, graph_views_[graph]);
        states_for_invisible_graphs_[graph->getUUID()] = doc["adapters"];

        ui->tabWidget->removeTab(page);

        visible_graphs_.erase(graph);
        graph_views_.erase(graph);
        view_graphs_.erase(view);
        auuid_views_.erase(graph_facade->getAbsoluteUUID());

        view_connections_.erase(view);
    }
}

void Designer::removeGraph(GraphFacade* graph_facade)
{
    for(auto it = graphs_.begin(); it != graphs_.end(); ++it) {
        if(it->second.get() == graph_facade) {
            SubgraphNode* graph = graph_facade->getSubgraphNode();
            graph_connections_.erase(graph);
            GraphView* view = graph_views_[graph];
            graph_views_.erase(graph);
            view_graphs_.erase(view);
            graphs_.erase(it);
            delete view;
            return;
        }
    }

}

void Designer::updateMinimap()
{
    GraphView* view = getVisibleGraphView();
    minimap_->display(view);
}

void Designer::showNotification(const Notification &notification)
{
    auto pos = named_notifications_.find(notification.auuid);
    if(pos == named_notifications_.end()) {
        if(notification.error == ErrorState::ErrorLevel::NONE) {
            // for now we only show error messages
            return;
        }

        NotificationWidget* widget = new NotificationWidget(notification, this);
        named_notifications_[notification.auuid] = widget;

        QObject::connect(widget, &NotificationWidget::activated, this, &Designer::focusOnNode);
        QObject::connect(widget, &NotificationWidget::timeout, [this, notification](){
            removeNotification(notification);
        });
        int y = 0;
        for(NotificationWidget* nw : sorted_notifications_) {
            y += nw->height();
        }

        widget->move(widget->pos().x(), y);

        sorted_notifications_.push_back(widget);

        widget->show();
    } else {
        pos->second->setNotification(notification);
    }
}

void Designer::removeNotification(const Notification &notification)
{
    NotificationWidget* widget = named_notifications_.at(notification.auuid);
    named_notifications_.erase(notification.auuid);

    int offset_y = 0;

    if(notification_animation_) {
        notification_animation_->stop();
        delete notification_animation_;
    }
    notification_animation_ = new QParallelAnimationGroup;

    for(auto it = sorted_notifications_.begin(); it != sorted_notifications_.end();) {
        NotificationWidget* nw = *it;
        if(nw == widget) {
            it = sorted_notifications_.erase(it);

        } else {
            QRect rect = nw->geometry();
            QRect rect_to = rect;

            rect_to.setY(std::max(0, offset_y));
            rect_to.setHeight(rect.height());

            QPropertyAnimation *animation = new QPropertyAnimation(nw, "geometry");
            animation->setDuration(500);
            animation->setStartValue(rect);
            animation->setEndValue(rect_to);
            animation->setEasingCurve(QEasingCurve::OutCubic);

            notification_animation_->addAnimation(animation);

            ++it;
            offset_y += nw->height();
        }
    }

    if(notification_animation_->animationCount() == 0) {
        delete notification_animation_;
        notification_animation_ = nullptr;
    } else {
        notification_animation_->start(QPropertyAnimation::KeepWhenStopped);
    }
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

    Command::Ptr del = view->deleteSelected();
    view_core_.execute(del);
}
void Designer::copySelected()
{
    GraphView* view = getVisibleGraphView();
    if(!view) {
        return;
    }

    view->copySelected();
}
void Designer::groupSelected()
{
    GraphView* view = getVisibleGraphView();
    if(!view) {
        return;
    }

    view->groupSelected();
}

void Designer::ungroupSelected()
{
    GraphView* view = getVisibleGraphView();
    if(!view) {
        return;
    }

    view->ungroupSelected();
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
        return graph_views_.at(core_.getRoot()->getSubgraphNode());
    }
    return current_view;
}

GraphView* Designer::getGraphView(const AUUID &uuid) const
{
    return auuid_views_.at(uuid);
}

DesignerScene* Designer::getVisibleDesignerScene() const
{
    GraphView* view = getVisibleGraphView();
    if(!view) {
        return nullptr;
    }
    return view->designerScene();
}

NodeAdapterFactory* Designer::getNodeAdapterFactory() const
{
    return view_core_.getNodeAdapterFactory().get();
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
    ui->tabWidget->blockSignals(true);

    auto graphs = graphs_;
    for(auto& pair : graphs) {
        removeGraph(pair.second.get());
    }

    graphs_.clear();
    visible_graphs_.clear();
    auuid_views_.clear();
    view_graphs_.clear();
    graph_connections_.clear();
    view_connections_.clear();

    states_for_invisible_graphs_.clear();

    stopObserving();
}

void Designer::reinitialize()
{
    ui->tabWidget->blockSignals(false);

    addGraph(core_.getRoot());
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

void Designer::focusOnNode(const AUUID &id)
{
    AUUID graph_id = id.parentAUUID();

    // show the parent graph
    showGraph(graph_id);

    // set the node in focus and center it
    GraphView* view = getGraphView(graph_id);

    view->focusOnNode(id.id());
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


void Designer::saveView(SubgraphNode* graph, YAML::Node &doc)
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

void Designer::loadView(SubgraphNode* graph, YAML::Node &doc)
{
    DesignerIO designerio;

    auto pos = graph_views_.find(graph);
    if(pos != graph_views_.end()) {
        designerio.loadBoxes(doc, pos->second);
    }
    states_for_invisible_graphs_[graph->getUUID()] = doc["adapters"];
}

void Designer::useProfiler(std::shared_ptr<Profiler> profiler)
{
    Profilable::useProfiler(profiler);

    for(const auto& pair : graph_views_) {
        GraphView* view = pair.second;
        view->useProfiler(profiler);
    }
}

/// MOC
#include "../../../include/csapex/view/designer/moc_designer.cpp"
