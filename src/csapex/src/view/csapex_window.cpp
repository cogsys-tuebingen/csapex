/// HEADER
#include <csapex/view/csapex_window.h>

/// COMPONENT
#include <csapex/command/command_factory.h>
#include <csapex/command/command.h>
#include <csapex/core/csapex_core.h>
#include <csapex/core/graphio.h>
#include <csapex/core/settings.h>
#include <csapex/factory/node_factory.h>
#include <csapex/factory/snippet_factory.h>
#include <csapex/info.h>
#include <csapex/manager/message_renderer_manager.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/tag.h>
#include <csapex/model/token_data.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/plugin/plugin_locator.h>
#include <csapex/profiling/profiler.h>
#include <csapex/profiling/timer.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/scheduling/thread_group.h>
#include <csapex/view/designer/designer.h>
#include <csapex/view/designer/designerio.h>
#include <csapex/view/designer/tutorial_tree_model.h>
#include <csapex/view/node/box.h>
#include <csapex/view/node/node_statistics.h>
#include <csapex/view/utility/clipboard.h>
#include <csapex/view/utility/html_delegate.h>
#include <csapex/view/utility/node_list_generator.h>
#include <csapex/view/utility/snippet_list_generator.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/view/widgets/activity_legend.h>
#include <csapex/view/widgets/activity_timeline.h>
#include <csapex/view/widgets/minimap_widget.h>
#include <csapex/view/widgets/profiling_widget.h>
#include <csapex/view/widgets/screenshot_dialog.h>
#include "ui_csapex_window.h"

/// SYSTEM
#include <iostream>
#include <QCloseEvent>
#include <QMessageBox>
#include <QFileDialog>
#include <QApplication>
#include <QTreeWidget>
#include <QSignalMapper>
#include <thread>
#include <QShortcut>
#include <QMimeData>
#include <QClipboard>
#include <QTextCodec>

using namespace csapex;

CsApexWindow::CsApexWindow(CsApexViewCore& view_core, QWidget *parent)
    : QMainWindow(parent), view_core_(view_core), core_(view_core.getCore()), cmd_dispatcher_(&view_core_.getCommandDispatcher()),
      root_(view_core_.getCore().getRoot()), executor_(*core_.getThreadPool()), profiler_(std::make_shared<Profiler>()),
      ui(new Ui::CsApexWindow), designer_(new Designer(view_core)), minimap_(designer_->getMinimap()),
      activity_legend_(new ActivityLegend), activity_timeline_(new ActivityTimeline),
      init_(false), style_sheet_watcher_(nullptr), plugin_locator_(core_.getPluginLocator())
{
    qRegisterMetaType < QImage > ("QImage");
    qRegisterMetaType < TokenPtr > ("Token::Ptr");
    qRegisterMetaType < TokenConstPtr > ("Token::ConstPtr");
    qRegisterMetaType < TokenDataPtr > ("TokenData::Ptr");
    qRegisterMetaType < TokenDataConstPtr > ("TokenData::ConstPtr");
    qRegisterMetaType < std::string > ("std::string");
    qRegisterMetaType < std::shared_ptr<const Interval> > ("std::shared_ptr<const Interval>");
    qRegisterMetaType < Notification > ("Notification");

    QObject::connect(activity_legend_, SIGNAL(nodeSelectionChanged(QList<NodeWorker*>)), activity_timeline_, SLOT(setSelection(QList<NodeWorker*>)));

    connections_.emplace_back(root_->nodeWorkerAdded.connect([this](NodeWorkerPtr n) { activity_legend_->startTrackingNode(n); }));
    connections_.emplace_back(root_->nodeRemoved.connect([this](NodeHandlePtr n) { activity_legend_->stopTrackingNode(n); }));

    QObject::connect(activity_legend_, SIGNAL(nodeAdded(NodeWorker*)), activity_timeline_, SLOT(addNode(NodeWorker*)));
    QObject::connect(activity_legend_, SIGNAL(nodeRemoved(NodeWorker*)), activity_timeline_, SLOT(removeNode(NodeWorker*)));

    QTextCodec *utfCodec = QTextCodec::codecForName("UTF-8");
    QTextCodec::setCodecForLocale(utfCodec);

    designer_->useProfiler(profiler_);
}

CsApexWindow::~CsApexWindow()
{
    connections_.clear();

    delete ui;
}


void CsApexWindow::construct()
{
    loadStyleSheet();

    ui->setupUi(this);

    setupTimeline();

    Graph* graph = root_->getGraph();

    setupDesigner();

    ui->actionPause->setChecked(executor_.isPaused());
    ui->menuBar->setVisible(true);

    QObject::connect(ui->actionSave, SIGNAL(triggered()), this, SLOT(save()));
    QObject::connect(ui->actionSaveAs, SIGNAL(triggered()), this,  SLOT(saveAs()));
    QObject::connect(ui->actionSaveAsCopy, SIGNAL(triggered()), this,  SLOT(saveAsCopy()));
    QObject::connect(ui->actionLoad, SIGNAL(triggered()), this,  SLOT(load()));
    QObject::connect(ui->actionReload, SIGNAL(triggered()), this,  SLOT(reload()));
    QObject::connect(ui->actionReset, SIGNAL(triggered()), this,  SLOT(reset()));
    QObject::connect(ui->actionClear, SIGNAL(triggered()), this,  SLOT(clear()));
    QObject::connect(ui->actionUndo, SIGNAL(triggered()), this,  SLOT(undo()));
    QObject::connect(ui->actionRedo, SIGNAL(triggered()), this,  SLOT(redo()));

    //    QObject::connect(ui->actionPause, SIGNAL(triggered(bool)), &core_, SLOT(setPause(bool)));
    QObject::connect(ui->actionPause, &QAction::triggered, [this](bool pause) { core_.setPause(pause); });

    QObject::connect(ui->actionSteppingMode, &QAction::triggered, [this](bool step) { core_.setSteppingMode(step); });
    QObject::connect(ui->actionStep, &QAction::triggered, [this](bool) { core_.step(); });


    QObject::connect(ui->actionClearBlock, SIGNAL(triggered(bool)), this, SLOT(clearBlock()));
    QObject::connect(ui->actionReset_Activity, SIGNAL(triggered(bool)), this, SLOT(resetActivity()));


    QObject::connect(designer_, SIGNAL(selectionChanged()), this, SLOT(updateSelectionActions()));
    updateSelectionActions();
    QObject::connect(QApplication::clipboard(), SIGNAL(dataChanged()), this, SLOT(updateClipboardActions()));

    QObject::connect(designer_, SIGNAL(selectionChanged()), this, SLOT(updateDebugInfo()));
    QObject::connect(designer_, SIGNAL(helpRequest(NodeBox*)), this, SLOT(showHelp(NodeBox*)));
    QObject::connect(ui->action_How_to_install, SIGNAL(triggered()), this, SLOT(showHowToInstall()));

    QObject::connect(ui->actionDelete_Selected, SIGNAL(triggered(bool)), designer_, SLOT(deleteSelected()));
    QObject::connect(ui->actionCopy, SIGNAL(triggered(bool)), designer_, SLOT(copySelected()));
    QObject::connect(ui->actionPaste, SIGNAL(triggered(bool)), designer_, SLOT(paste()));

    QObject::connect(ui->actionCreate_Node, SIGNAL(triggered(bool)), designer_, SLOT(showNodeDialog()));
    QObject::connect(ui->actionFind_Node, SIGNAL(triggered(bool)), designer_, SLOT(showNodeSearchDialog()));

    QObject::connect(ui->actionGroup, SIGNAL(triggered(bool)), designer_, SLOT(groupSelected()));
    QObject::connect(ui->actionUngroup, SIGNAL(triggered(bool)), designer_, SLOT(ungroupSelected()));

    QObject::connect(ui->actionClear_selection, SIGNAL(triggered()), designer_,  SLOT(clearSelection()));
    QObject::connect(ui->actionSelect_all, SIGNAL(triggered()), designer_,  SLOT(selectAll()));

    QObject::connect(ui->actionMake_Screenshot, SIGNAL(triggered()), this, SLOT(makeScreenshot()));

    QObject::connect(ui->actionAbout_CS_APEX, SIGNAL(triggered()), this, SLOT(about()));
    QObject::connect(ui->actionCopyright_Notices, SIGNAL(triggered()), this, SLOT(copyRight()));

    QObject::connect(ui->node_info_tree, SIGNAL(itemSelectionChanged()), this, SLOT(updateNodeInfo()));

    QObject::connect(ui->profiling_debug_enable, SIGNAL(toggled(bool)), this, SLOT(enableDebugProfiling(bool)));

    connections_.push_back(core_.resetRequest.connect([this](){ designer_->reset(); }));
    connections_.push_back(core_.resetDone.connect([this](){ designer_->reinitialize(); }));
    connections_.push_back(core_.configChanged.connect([this](){ updateTitle(); }));
    connections_.push_back(core_.showStatusMessage.connect([this](const std::string& status){ showStatusMessage(status); }));
    connections_.push_back(core_.newNodeType.connect([this](){ updateNodeTypes(); }));
    connections_.push_back(core_.newSnippetType.connect([this](){ updateSnippets(); }));

    connections_.push_back(graph->state_changed.connect([this]() { updateMenu(); }));
    connections_.push_back(root_->panic.connect([this]() { clearBlock(); }));

    connections_.push_back(cmd_dispatcher_->stateChanged.connect([this](){ updateUndoInfo(); }));
    connections_.push_back(cmd_dispatcher_->dirtyChanged.connect([this](bool) { updateTitle(); }));

    connections_.push_back(core_.paused.connect([this](bool pause) { ui->actionPause->setChecked(pause); }));

    QObject::connect(this, &CsApexWindow::showNotificationRequest, this, &CsApexWindow::showNotification);
    connections_.push_back(core_.notification.connect([this](Notification notification){ showNotificationRequest(notification); }));

    connections_.push_back(core_.begin_step.connect([this](){ ui->actionStep->setEnabled(false); }));
    connections_.push_back(core_.end_step.connect([this](){ ui->actionStep->setEnabled(core_.isSteppingMode()); }));

    connections_.push_back(core_.getThreadPool()->group_created.connect([this](const ThreadGroupPtr& group) { updateThreadInfo(); }));

    updateMenu();
    updateTitle();

    createPluginsMenu();
    createTutorialsMenu();

    updateNodeInfo();

    timer.setInterval(100);
    timer.setSingleShot(false);
    timer.start();

    QObject::connect(&timer, SIGNAL(timeout()), this, SLOT(tick()));
}


void CsApexWindow::setupDesigner()
{
    designer_->setup();
    setCentralWidget(designer_);

    DesignerOptions* opt = designer_->options();

    ui->actionGrid->setChecked(opt->isGridEnabled());
    ui->actionSchematics->setChecked(opt->isSchematicsEnabled());
    ui->actionLock_to_Grid->setChecked(opt->isGridLockEnabled());
    ui->actionDisplay_Graph_Components->setChecked(opt->isGraphComponentsEnabled());
    ui->actionDisplay_Threads->setChecked(opt->isThreadsEnabled());
    ui->actionDisplay_Frequency->setChecked(opt->isFrequencyEnabled());

    ui->actionSignal_Connections->setChecked(opt->areSignalConnectionsVisible());
    ui->actionMessage_Connections->setChecked(opt->areMessageConnectionsVisibile());

    ui->actionDebug->setChecked(opt->isDebug());

    minimap_->setVisible(opt->isMinimapEnabled());
    ui->actionDisplay_Minimap->setChecked(opt->isMinimapEnabled());

    QObject::connect(ui->actionGrid, SIGNAL(toggled(bool)), opt,  SLOT(enableGrid(bool)));
    QObject::connect(opt, SIGNAL(gridEnabled(bool)), ui->actionGrid, SLOT(setChecked(bool)));
    QObject::connect(ui->actionSchematics, SIGNAL(toggled(bool)), opt,  SLOT(enableSchematics(bool)));
    QObject::connect(opt, SIGNAL(schematicsEnabled(bool)), ui->actionSchematics, SLOT(setChecked(bool)));
    QObject::connect(ui->actionDisplay_Graph_Components, SIGNAL(toggled(bool)), opt,  SLOT(displayGraphComponents(bool)));
    QObject::connect(opt, SIGNAL(graphComponentsEnabled(bool)), ui->actionDisplay_Graph_Components, SLOT(setChecked(bool)));
    QObject::connect(ui->actionDisplay_Threads, SIGNAL(toggled(bool)), opt,  SLOT(displayThreads(bool)));
    QObject::connect(opt, SIGNAL(threadsEnabled(bool)), ui->actionDisplay_Threads, SLOT(setChecked(bool)));
    QObject::connect(ui->actionDisplay_Frequency, SIGNAL(toggled(bool)), opt,  SLOT(displayFrequency(bool)));
    QObject::connect(opt, SIGNAL(frequencyEnabled(bool)), ui->actionDisplay_Frequency, SLOT(setChecked(bool)));
    QObject::connect(ui->actionDisplay_Minimap, SIGNAL(toggled(bool)), opt,  SLOT(displayMinimap(bool)));
    QObject::connect(opt, SIGNAL(minimapEnabled(bool)), ui->actionDisplay_Minimap, SLOT(setChecked(bool)));

    QObject::connect(ui->actionSignal_Connections, SIGNAL(toggled(bool)), opt, SLOT(displaySignalConnections(bool)));
    QObject::connect(opt, SIGNAL(signalsEnabled(bool)), ui->actionSignal_Connections, SLOT(setChecked(bool)));

    QObject::connect(ui->actionMessage_Connections, SIGNAL(toggled(bool)), opt, SLOT(displayMessageConnections(bool)));
    QObject::connect(opt, SIGNAL(messagesEnabled(bool)), ui->actionMessage_Connections, SLOT(setChecked(bool)));

    QObject::connect(ui->actionDebug, SIGNAL(toggled(bool)), opt, SLOT(enableDebug(bool)));
    QObject::connect(opt, SIGNAL(debugEnabled(bool)), ui->actionDebug, SLOT(setChecked(bool)));

    QObject::connect(ui->actionLock_to_Grid, SIGNAL(toggled(bool)),opt,  SLOT(enableGridLock(bool)));
    QObject::connect(opt, SIGNAL(gridLockEnabled(bool)), ui->actionLock_to_Grid, SLOT(setChecked(bool)));

    ui->startup->layout()->addWidget(new ProfilingWidget(core_.getProfiler(), "load graph"));
}


void CsApexWindow::setupTimeline()
{
    ui->timeline->layout()->setAlignment(Qt::AlignLeft | Qt::AlignTop);

    QHBoxLayout* layout = dynamic_cast<QHBoxLayout*>(ui->timeline->layout());

    layout->addWidget(activity_legend_, 0, Qt::AlignTop);
    layout->addWidget(activity_timeline_, 0, Qt::AlignTop);

    QObject::connect(ui->timeline_reset, SIGNAL(pressed()), activity_timeline_, SLOT(reset()));

    QObject::connect(ui->timeline_record, SIGNAL(toggled(bool)), activity_timeline_, SLOT(setRecording(bool)));
    QObject::connect(activity_timeline_, SIGNAL(recordingChanged(bool)), ui->timeline_record, SLOT(setChecked(bool)));
}

void CsApexWindow::updateSelectionActions()
{
    bool enabled = designer_->hasSelection();
    ui->actionDelete_Selected->setEnabled(enabled);
    ui->actionCopy->setEnabled(enabled);
    ui->actionClear_selection->setEnabled(enabled);
    ui->actionGroup->setEnabled(enabled);

    std::vector<NodeBox*> selected = designer_->getSelectedBoxes();
    bool is_graph = selected.size() == 1 && selected[0]->getNodeHandle()->getType() == "csapex::Graph";
    ui->actionUngroup->setEnabled(is_graph);
}

void CsApexWindow::updateClipboardActions()
{
    ui->actionPaste->setEnabled(ClipBoard::canPaste());
}

void CsApexWindow::showHelp(NodeBox *box)
{
    if(ui->HelpCenter->isHidden()) {
        ui->HelpCenter->show();
    }


    std::string node_type = box->getNodeHandle()->getType();

    QTreeWidgetItemIterator it(ui->node_info_tree);
    while (*it) {
        QTreeWidgetItem* item = *it;

        std::string type = item->data(0, Qt::UserRole + 1).toString().toStdString();

        if(type == node_type) {
            ui->node_info_tree->clearSelection();
            ui->node_group->setChecked(false);
            QTreeWidgetItem* tmp = *it;
            tmp->setSelected(true);
            do {
                ui->node_info_tree->expandItem(tmp);
                tmp = tmp->parent();
            }  while(tmp);
            return;
        }
        ++it;
    }
}

void CsApexWindow::showHowToInstall()
{
    ui->HelpCenter->show();
    ui->help_toolbox->show();
    ui->help_toolbox->setCurrentWidget(ui->plugins);
}

void CsApexWindow::updateDebugInfo()
{
    if(!ui->Debug->isVisible()) {
        return;
    }

    ui->box_info->clear();

    std::vector<NodeBox*> selected = designer_->getSelectedBoxes();

    for(NodeBox* box : selected) {
        NodeHandle* handle = box->getNodeHandle();
        handle->nodeStateChanged.connect([this](){ updateDebugInfo(); });
        ui->box_info->addTopLevelItem(NodeStatistics(handle).createDebugInformation(&core_.getNodeFactory()));
    }

    QTreeWidgetItemIterator it(ui->box_info);
    while (*it) {
        QTreeWidgetItem* item = *it;
        bool expand = item->data(0, Qt::UserRole).toBool();

        int depth = 0;
        while(item->parent()) {
            item = item->parent();
            ++depth;
        }

        if(depth <= 1 || expand) {
            QTreeWidgetItem* item = *it;
            do {
                ui->box_info->expandItem(item);
                item = item->parent();
            }  while(item);
        }
        ++it;
    }

    for(int i = 0; i < ui->box_info->depth(); ++i) {
        ui->box_info->resizeColumnToContents(i);
    }
}

void CsApexWindow::updateNodeInfo()
{
    std::stringstream ss;

    for(QTreeWidgetItem* item : ui->node_info_tree->selectedItems()) {
        QString type = item->data(0, Qt::UserRole + 1).toString();
        if(!type.isEmpty()) {
            NodeConstructor::Ptr n = core_.getNodeFactory().getConstructor(type.toStdString());

            QString icon = QString::fromStdString(n->getIcon());
            QImage image = QIcon(icon).pixmap(QSize(16,16)).toImage();
            QUrl uri ( QString::fromStdString(n->getType()));
            ui->node_info_text->document()->addResource( QTextDocument::ImageResource, uri, QVariant ( image ) );

            ss << "<h1> <img src=\"" << uri.toString().toStdString() << "\" /> " << n->getType() << "</h1>";
            ss << "<p>Tags: <i>";
            int i = 0;
            for(const Tag::Ptr& tag : n->getTags()) {
                if(i++ > 0) {
                    ss << ", ";
                }
                ss << tag->getName();
            }
            ss << "</p>";
            ss << "<h3>" << n->getDescription() << "</h3>";

            ss << "<hr />";
            ss << "<h1>Parameters:</h1>";


            auto node = n->makePrototype()->getNode().lock();
            if(node) {
                node->setupParameters(*node);
                std::vector<csapex::param::Parameter::Ptr> params = node->getParameters();

                for(const csapex::param::Parameter::Ptr& p : params) {
                    if(!p->isHidden()) {
                        ss << "<h2>" << p->name() << "</h2>";
                        ss << "<p>" << p->description().toString() << "</p>";
                        ss << "<p>" << p->toString() << "</p>";
                    }
                }
            }
        }
    }


    ui->node_info_text->setHtml(QString::fromStdString(ss.str()));
}

void CsApexWindow::updateThreadInfo()
{
    ThreadPoolPtr thread_pool = core_.getThreadPool();

    const std::vector<ThreadGroupPtr>& threads = thread_pool->getGroups();

    ui->thread_table->setRowCount(threads.size());
    ui->thread_table->setColumnCount(3);


    int row = 0;
    for(const ThreadGroupPtr& thread : threads) {
        QTableWidgetItem* main_item = new QTableWidgetItem(QString::fromStdString(thread->getName()));
        main_item->setText(QString::fromStdString(thread->getName()));
        main_item->setTextAlignment(Qt::AlignVCenter);

        if(thread->id() == ThreadGroup::PRIVATE_THREAD) {
            main_item->setIcon(QIcon(QPixmap(":/thread_group_none.png")));
        } else {
            main_item->setIcon(QIcon(QPixmap(":/thread_group.png")));
        }

        QWidget* widget = nullptr;

        ui->thread_table->setItem(row, 0, main_item);
        ui->thread_table->setCellWidget(row, 1, widget);

        ++row;
    }

    QObject::disconnect(ui->thread_table);

//    QObject::connect(ui->thread_table, &QTableWidget::keyPressEvent,
//                     [this](QKeyEvent* key) {
//        if(key->key() == Qt::Key_Delete) {
//            QList<int> rows;
//            QItemSelection selection( ui->thread_table->selectionModel()->selection() );
//            foreach( const QModelIndex & index, selection.indexes() ) {
//               rows.append( index.row() );
//            }

//            qSort( rows );

//            int prev = -1;
//            for( int i = rows.count() - 1; i >= 0; i -= 1 ) {
//               int current = rows[i];
//               if( current != prev ) {
//                  ui->thread_table->model()->removeRows( current, 1 );
//                  prev = current;
//               }
//            }
//        }
//    });

//    QObject::connect(ui->thread_table, &QTableWidget::cellChanged,
//                     [this](int row, int col) {

//        auto item = ui->thread_table->itemAt(row, col);
//        ThreadPoolPtr thread_pool = core_.getThreadPool();

//        if(col == 0) {
//            // name changed
//            QString new_name = item->data(Qt::EditRole).toString();

//            ThreadGroupPtr group = thread_pool->getGroups()[row];
//            if(new_name.isEmpty()) {
//                item->setData(Qt::EditRole, QString::fromStdString(group->getName()));

//            } else {
//                // command
//                group->setName(new_name.toStdString());
//            }

//        }
//    });
}

void CsApexWindow::updateUndoInfo()
{
    ui->undo->clear();
    ui->redo->clear();

    std::deque<QTreeWidgetItem*> stack;

    auto iterator = [&stack](QTreeWidget* tree, int level, const Command& cmd) {
        while(level < (int) stack.size()) {
            stack.pop_back();
        }
        QTreeWidgetItem* tl = new QTreeWidgetItem;
        tl->setText(0, cmd.getType().c_str());
        tl->setText(1, cmd.getDescription().c_str());

        if(level == 0) {
            tree->addTopLevelItem(tl);
        } else {
            stack.back()->addChild(tl);
        }

        stack.push_back(tl);
    };

    cmd_dispatcher_->visitUndoCommands([this, &iterator](int level, const Command& cmd) {
        iterator(ui->undo, level, cmd);
    });
    stack.clear();
    cmd_dispatcher_->visitRedoCommands([this, &iterator](int level, const Command& cmd) {
        iterator(ui->redo, level, cmd);
    });

    ui->undo->expandAll();
    ui->redo->expandAll();
}

void CsApexWindow::about()
{
    std::stringstream ss;
    ss << "<h1>cs::APEX " << csapex::info::CSAPEX_VERSION << "</h1>";
    ss << "<p>Based on QT " << QT_VERSION_STR ;
#ifdef __GNUC__
    ss << " (GCC " << __VERSION__ << ")";
#endif
    ss << "</p>";
    ss << "<p>Built on " << __DATE__ << " at " << __TIME__ << "</p>";
    ss << "<p>From revision " << csapex::info::GIT_COMMIT_HASH << " (" << csapex::info::GIT_BRANCH << ")</p>";
    ss << "<p>The program is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.</p>";

    QMessageBox::about(this, "About cs::APEX", ss.str().c_str());
}

void CsApexWindow::copyRight()
{
    QString copyright =
            "<p>"
            "<h1>UI</h1>"
            "<h2>Icons</h2>"
            "<h3>Silk icon set 1.3</h3>"
            "<div>Made by Mark James, available from <a href=\"http://www.famfamfam.com/lab/icons/silk/\">famfamfam.com</a> "
            "is licensed under <a href=\"http://creativecommons.org/licenses/by/2.5/\" title=\"Creative Commons BY 2.5\">CC BY 2.5</a>"
            "</div>"
            "<h3>Rounded UI</h3>"
            "<div>Icon made by <a href=\"http://superstoked.se\" title=\"Robin Kylander\">Robin Kylander</a> "
            "from <a href=\"http://www.flaticon.com\" title=\"Flaticon\">www.flaticon.com</a> "
            "is licensed under <a href=\"http://creativecommons.org/licenses/by/3.0/\" title=\"Creative Commons BY 3.0\">CC BY 3.0</a>"
            "</div>"
            "</p>"
            ;

    QMessageBox::information(this, "Copyright", copyright);
}

void CsApexWindow::clearBlock()
{
    std::cerr << "clearing blocking connections" << std::endl;
    root_->clearBlock();
}

void CsApexWindow::resetActivity()
{
    std::cerr << "resetting activity" << std::endl;
    root_->resetActivity();
}


void CsApexWindow::enableDebugProfiling(bool enabled)
{
    profiler_->setEnabled(enabled);

    if(enabled) {
        QLayout* layout = ui->profiling_debug_profilers->layout();
        if(!layout) {
            layout = new QVBoxLayout();

            layout->addWidget(new ProfilingWidget(profiler_, "drawForeground"));
            layout->addWidget(new ProfilingWidget(profiler_, "drawBackground"));

            ui->profiling_debug_profilers->setLayout(layout);
        }
    }
}

void CsApexWindow::updateNodeTypes()
{
    if(ui->nodes->layout()) {
        QtHelper::clearLayout(ui->nodes->layout());
    } else {
        ui->nodes->setLayout(new QVBoxLayout);
    }
    if(ui->node_info_tree->layout()) {
        QtHelper::clearLayout(ui->node_info_tree->layout());
    } else {
        ui->node_info_tree->setLayout(new QVBoxLayout);
    }

    NodeListGenerator generator(core_.getNodeFactory(), *designer_->getNodeAdapterFactory());

    generator.insertAvailableNodeTypes(ui->nodes);
    generator.insertAvailableNodeTypes(ui->node_info_tree);
}

void CsApexWindow::updateSnippets()
{
    SnippetListGenerator generator(core_.getSnippetFactory());
    ui->snippets->clear();
    generator.insertAvailableSnippets(ui->snippets);
}


void CsApexWindow::loadStyleSheet(const QString& path)
{
    QFile file(path);
    file.open(QFile::ReadOnly);
    QString style_sheet(file.readAll());
    QWidget::setStyleSheet(style_sheet);

    designer_->overwriteStyleSheet(style_sheet);

    delete style_sheet_watcher_;
    style_sheet_watcher_ = nullptr;

    style_sheet_watcher_ = new QFileSystemWatcher(this);
    style_sheet_watcher_->addPath(path);

    QObject::connect(style_sheet_watcher_, SIGNAL(fileChanged(const QString&)),
                     this, SLOT(reloadStyleSheet(const QString&)));
}


void CsApexWindow::reloadStyleSheet(const QString& path)
{
    QFile qfile(path);
    if(qfile.exists()) {
        std::chrono::milliseconds dura(100);

        std::this_thread::sleep_for(dura);
        while(qfile.size() == 0) {
            std::this_thread::sleep_for(dura);
        }
        loadStyleSheet(path);
    }
}

void CsApexWindow::loadStyleSheet()
{
    std::string cfg = Settings::defaultConfigPath() + "cfg/style.css";

    loadStyleSheet(cfg.c_str());
}


void CsApexWindow::start()
{
    showStatusMessage("building ui");
    construct();
    showStatusMessage("initializing");
    init();

    designer_->setFocus(Qt::OtherFocusReason);
}

void CsApexWindow::updateMenu()
{
    bool can_undo = cmd_dispatcher_->canUndo();
    ui->actionUndo->setDisabled(!can_undo);
    if(can_undo) {
        ui->actionUndo->setText(QString("&Undo ") + QString::fromStdString(cmd_dispatcher_->getNextUndoCommand()->getType()));
    } else {
        ui->actionUndo->setText(QString("&Undo"));
    }

    bool can_redo = cmd_dispatcher_->canRedo();
    ui->actionRedo->setDisabled(!can_redo);
    if(can_redo) {
        ui->actionRedo->setText(QString("&Redo ") + QString::fromStdString(cmd_dispatcher_->getNextRedoCommand()->getType()));
    } else {
        ui->actionRedo->setText(QString("&Redo"));
    }

}

void CsApexWindow::updateTitle()
{
    std::stringstream window;
    window << "CS::APEX (" << getConfigFile() << ")";

    if(cmd_dispatcher_->isDirty()) {
        window << " *";
    }

    bool recovery = core_.getSettings().get<bool>("config_recovery", false);
    if(recovery) {
        window << " (recovery)";
    }

    setWindowTitle(window.str().c_str());
}

void CsApexWindow::createPluginsMenu()
{
    std::vector<std::string> plugins = plugin_locator_->getAllLibraries();

    for(const std::string& library : plugins) {
        /// enable / ignore plugin
        QAction* enable_ignore = new QAction(QString::fromStdString(library), ui->menu_Available_Plugins);
        enable_ignore->setCheckable(true);
        enable_ignore->setObjectName(QString::fromStdString(library));

        bool ignored = plugin_locator_->isLibraryIgnored(library);
        bool error = plugin_locator_->hasLibraryError(library);

        enable_ignore->setChecked(!ignored);
        if(ignored) {
            enable_ignore->setIcon(QIcon(":/plugin_disabled.png"));
        } else if(error) {
            enable_ignore->setIcon(QIcon(":/plugin_error.png"));
            enable_ignore->setToolTip(QString::fromStdString(plugin_locator_->getLibraryError(library)));
        } else {
            enable_ignore->setIcon(QIcon(":/plugin.png"));
        }
        ui->menu_Available_Plugins->addAction(enable_ignore);

        QSignalMapper* enable_ignore_mapper = new QSignalMapper(this);
        QObject::connect(enable_ignore, SIGNAL(toggled(bool)), enable_ignore_mapper, SLOT(map()));
        enable_ignore_mapper->setMapping(enable_ignore, enable_ignore);

        QObject::connect(enable_ignore_mapper, SIGNAL(mapped(QObject*)), this, SLOT(updatePluginIgnored(QObject*)));
    }
}

void CsApexWindow::createTutorialsMenu()
{
    QTreeWidget* tree = ui->tutorials_tree;
    tree->setWordWrap(true);
    tree->setUniformRowHeights(false);
    tree->setItemDelegate(new HTMLDelegate);

    TutorialTreeModel tutorials(core_.getSettings());
    tutorials.fill(tree);

    QObject::connect(tree, &QTreeWidget::activated,
                     this, &CsApexWindow::loadTutorial);
//    QObject::connect(tree, &QTreeWidget::doubleClicked,
//                     this, &CsApexWindow::loadTutorial);
}

void CsApexWindow::loadTutorial(const QModelIndex &index)
{
    QTreeWidget* tree = ui->tutorials_tree;
    QVariant data = tree->model()->data(index, Qt::UserRole);
    if(data.isValid()) {
        QString filename = data.toString();
        if(QFile(filename).exists()) {
            core_.load(filename.toStdString());
        }
    }
}

void CsApexWindow::updatePluginIgnored(const QObject* &action)
{
    const QAction* a = dynamic_cast<const QAction*>(action);

    plugin_locator_->ignoreLibrary(a->objectName().toStdString(), !a->isChecked());
}


void CsApexWindow::tick()
{
    cmd_dispatcher_->executeLater();
}

void CsApexWindow::closeEvent(QCloseEvent* event)
{
    if(cmd_dispatcher_->isDirty()) {
        int r = QMessageBox::warning(this, tr("cs::APEX"),
                                     tr("Do you want to save the layout before closing?"),
                                     QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        if(r == QMessageBox::Save) {
            save();
            event->accept();
        } else if(r == QMessageBox::Discard) {
            event->accept();
        } else {
            event->ignore();
            return;
        }
    }

    QString geometry(saveGeometry().toBase64());
    QString uistate(saveState().toBase64());

    Settings& settings = core_.getSettings();
    if(!settings.knows("uistate")) {
        settings.add(csapex::param::ParameterFactory::declareText("uistate", ""));
    }
    if(!settings.knows("geometry")) {
        settings.add(csapex::param::ParameterFactory::declareText("geometry", geometry.toStdString()));
    }

    settings.set("uistate", uistate.toStdString());
    settings.set("geometry", geometry.toStdString());
    core_.settingsChanged();

    try {
        root_->stop();
    } catch(const std::exception& e) {
        std::cerr << "exception while stopping graph worker: " << e.what() << std::endl;
    } catch(...) {
        throw;
    }

    event->accept();
    closed();
}

void CsApexWindow::showStatusMessage(const std::string &msg)
{
    Q_EMIT statusChanged(QString(msg.c_str()));
}

void CsApexWindow::showNotification(const Notification &notification)
{
    designer_->showNotification(notification);
    statusBar()->showMessage(QString::fromStdString(notification.auuid.getFullName() + ": " + notification.message));
}

void CsApexWindow::init()
{
    init_ = true;

    updateNodeTypes();
    updateSnippets();
    //    designer_->show();

    Settings& settings = core_.getSettings();
    if(settings.knows("uistate")) {
        std::string uistate = settings.get<std::string>("uistate");
        restoreState(QByteArray::fromBase64(uistate.data()));
    } else {
        for(auto dw : findChildren<QDockWidget*>()) {
            dw->hide();
        }
    }
    if(settings.knows("geometry")) {
        std::string geometry = settings.get<std::string>("geometry");
        restoreGeometry(QByteArray::fromBase64(geometry.data()));
    }
}

std::string CsApexWindow::getConfigFile()
{
    return core_.getSettings().get<std::string>("config");
}

void CsApexWindow::save()
{
    core_.saveAs(getConfigFile());
}

void CsApexWindow::saveAs()
{
    QString filename = QFileDialog::getSaveFileName(0, "Save config", QString::fromStdString(getConfigFile()),
                                                    QString::fromStdString(Settings::config_selector), 0, QFileDialog::DontUseNativeDialog);

    if(!filename.isEmpty()) {
        core_.saveAs(filename.toStdString());
        core_.getSettings().set("config", filename.toStdString());
    }
}


void CsApexWindow::saveAsCopy()
{
    QString filename = QFileDialog::getSaveFileName(0, "Save config", QString::fromStdString(getConfigFile()),
                                                    QString::fromStdString(Settings::config_selector), 0, QFileDialog::DontUseNativeDialog);

    if(!filename.isEmpty()) {
        core_.saveAs(filename.toStdString());
    }
}

void CsApexWindow::reload()
{
    core_.load(getConfigFile());
}

void CsApexWindow::reset()
{
    int r = QMessageBox::warning(this, tr("cs::APEX"),
                                 tr("Do you really want to reset? This <b>cannot</b> be undone!"),
                                 QMessageBox::Ok | QMessageBox::Cancel);
    if(r == QMessageBox::Ok) {
        core_.reset();
    }
}

void CsApexWindow::clear()
{
    CommandPtr cmd = CommandFactory(root_.get()).clearCommand();
    cmd_dispatcher_->execute(cmd);
}

void CsApexWindow::undo()
{
    cmd_dispatcher_->undo();
}

void CsApexWindow::redo()
{
    cmd_dispatcher_->redo();
}

void CsApexWindow::makeScreenshot()
{
    ScreenshotDialog diag(root_, this);
    diag.exec();
}

void CsApexWindow::load()
{
    QString filename = QFileDialog::getOpenFileName(0, "Load config", QString::fromStdString(getConfigFile()),
                                                    QString::fromStdString(Settings::config_selector), 0, QFileDialog::DontUseNativeDialog);

    if(QFile(filename).exists()) {
        core_.load(filename.toStdString());
    }
}

/// MOC
#include "../../include/csapex/view/moc_csapex_window.cpp"
