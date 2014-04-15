/// HEADER
#include <csapex/view/csapex_window.h>

/// COMPONENT
#include <csapex/command/dispatcher.h>
#include <csapex/core/designerio.h>
#include <csapex/core/drag_io.h>
#include <csapex/core/graphio.h>
#include <csapex/core/settings.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/utility/bash_parser.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/view/box.h>
#include <csapex/view/design_board.h>
#include <csapex/view/designer.h>
#include "ui_csapex_window.h"
#include <csapex/view/widget_controller.h>

/// SYSTEM
#include <iostream>
#include <QCloseEvent>
#include <QMessageBox>
#include <QToolBar>
#include <QTimer>
#include <QFileDialog>
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QDesktopWidget>

using namespace csapex;

CsApexWindow::CsApexWindow(CsApexCore& core, CommandDispatcher* cmd_dispatcher, WidgetControllerPtr widget_ctrl, GraphPtr graph, Designer* designer, QWidget *parent)
    : QMainWindow(parent), core_(core), cmd_dispatcher_(cmd_dispatcher), widget_ctrl_(widget_ctrl), graph_(graph), ui(new Ui::EvaluationWindow), designer_(designer), init_(false), style_sheet_watcher_(NULL)
{
    core_.addListener(this);
}

CsApexWindow::~CsApexWindow()
{
    core_.removeListener(this);
}

void CsApexWindow::construct()
{
    loadStyleSheet();

    ui->setupUi(this);

    designer_->hide();
    ui->splitter->addWidget(designer_);
    ui->splitter->addWidget(ui->logOutput);

    Graph* graph = graph_.get();

    ui->actionGrid->setChecked(designer_->isGridEnabled());
    ui->actionLock_to_Grid->setChecked(designer_->isGridLockEnabled());

    QObject::connect(ui->actionSave, SIGNAL(triggered()), this, SLOT(save()));
    QObject::connect(ui->actionSaveAs, SIGNAL(triggered()), this,  SLOT(saveAs()));
    QObject::connect(ui->actionSaveAsCopy, SIGNAL(triggered()), this,  SLOT(saveAsCopy()));
    QObject::connect(ui->actionLoad, SIGNAL(triggered()), this,  SLOT(load()));
    QObject::connect(ui->actionReload, SIGNAL(triggered()), this,  SLOT(reload()));
    QObject::connect(ui->actionReset, SIGNAL(triggered()), this,  SLOT(reset()));
    QObject::connect(ui->actionClear, SIGNAL(triggered()), this,  SLOT(clear()));
    QObject::connect(ui->actionUndo, SIGNAL(triggered()), this,  SLOT(undo()));
    QObject::connect(ui->actionRedo, SIGNAL(triggered()), this,  SLOT(redo()));

    QObject::connect(ui->actionPause, SIGNAL(triggered(bool)), &core_, SLOT(setPause(bool)));
    QObject::connect(&core_, SIGNAL(paused(bool)), ui->actionPause, SLOT(setChecked(bool)));

    QObject::connect(ui->actionGrid, SIGNAL(toggled(bool)), designer_,  SLOT(enableGrid(bool)));
    QObject::connect(designer_, SIGNAL(gridEnabled(bool)), ui->actionGrid, SLOT(setChecked(bool)));
    QObject::connect(ui->actionLock_to_Grid, SIGNAL(toggled(bool)), designer_,  SLOT(lockToGrid(bool)));
    QObject::connect(designer_, SIGNAL(gridLockEnabled(bool)), ui->actionLock_to_Grid, SLOT(setChecked(bool)));

    QObject::connect(ui->actionDelete_Selected, SIGNAL(triggered(bool)), designer_, SLOT(deleteSelected()));
    QObject::connect(&widget_ctrl_->box_selection_, SIGNAL(selectionChanged()), this, SLOT(updateDeleteAction()));
    QObject::connect(&widget_ctrl_->connection_selection_, SIGNAL(selectionChanged()), this, SLOT(updateDeleteAction()));

    QObject::connect(&widget_ctrl_->box_selection_, SIGNAL(selectionChanged()), designer_, SLOT(updateDebugInfo()));
    QObject::connect(&widget_ctrl_->connection_selection_, SIGNAL(selectionChanged()), designer_, SLOT(updateDebugInfo()));

    QObject::connect(ui->actionClear_selection, SIGNAL(triggered()), &widget_ctrl_->box_selection_,  SLOT(clearSelection()));
    QObject::connect(ui->actionSelect_all, SIGNAL(triggered()), &widget_ctrl_->box_selection_,  SLOT(selectAll()));

    QObject::connect(graph, SIGNAL(stateChanged()), designer_, SLOT(stateChangedEvent()));
    QObject::connect(graph, SIGNAL(stateChanged()), this, SLOT(updateMenu()));

    QObject::connect(&core_, SIGNAL(configChanged()), this, SLOT(updateTitle()));
    QObject::connect(&core_, SIGNAL(showStatusMessage(const std::string&)), this, SLOT(showStatusMessage(const std::string&)));
    QObject::connect(&core_, SIGNAL(reloadBoxMenues()), this, SLOT(reloadBoxMenues()));
    QObject::connect(&core_, SIGNAL(saveSettingsRequest(YAML::Emitter&)), this, SLOT(saveSettings(YAML::Emitter&)));
    QObject::connect(&core_, SIGNAL(loadSettingsRequest(YAML::Node&)), this, SLOT(loadSettings(YAML::Node&)));

    QObject::connect(graph, SIGNAL(nodeAdded(NodePtr)), widget_ctrl_.get(), SLOT(nodeAdded(NodePtr)));
    QObject::connect(graph, SIGNAL(nodeRemoved(NodePtr)), widget_ctrl_.get(), SLOT(nodeRemoved(NodePtr)));

    QObject::connect(graph, SIGNAL(dirtyChanged(bool)), this, SLOT(updateTitle()));

    updateMenu();
    updateTitle();

    hideLog();

    timer.setInterval(100);
    timer.setSingleShot(false);
    timer.start();

    QObject::connect(&timer, SIGNAL(timeout()), this, SLOT(tick()));
}

void CsApexWindow::updateDeleteAction()
{
    bool has_selection = widget_ctrl_->connection_selection_.countSelected() + widget_ctrl_->box_selection_.countSelected() > 0;
    ui->actionDelete_Selected->setEnabled(has_selection);
}

void CsApexWindow::resetSignal()
{
    designer_->reset();
}

void CsApexWindow::loadStyleSheet(const QString& path)
{
    std::cout << "loading stylesheet " << path.toStdString() << std::endl;

    QFile file(path);
    file.open(QFile::ReadOnly);
    style_sheet_ = QString(file.readAll());
    QWidget::setStyleSheet(style_sheet_);
    BoxManager::instance().setStyleSheet(style_sheet_);

    if(style_sheet_watcher_) {
        delete style_sheet_watcher_;
    }

    style_sheet_watcher_ = new QFileSystemWatcher(this);
    style_sheet_watcher_->addPath(path);

    QObject::connect(style_sheet_watcher_, SIGNAL(fileChanged(const QString&)),
             this, SLOT(loadStyleSheet(const QString&)));
}

void CsApexWindow::loadStyleSheet()
{
    std::string cfg = Settings::defaultConfigPath() + "cfg/style.css";

    loadStyleSheet(cfg.c_str());
}

void CsApexWindow::showMenu()
{
    QVBoxLayout* new_layout = new QVBoxLayout;

    QToolBar* tb = new QToolBar;
    QMenuBar* mb = menuBar();
    tb->addActions(mb->actions());

    new_layout->addWidget(tb);

    QLayout* layout = ui->centralwidget->layout();
    QLayoutItem* item;
    while((item = layout->takeAt(0)) != NULL) {
        new_layout->addItem(item);
    }

    delete layout;
    ui->centralwidget->setLayout(new_layout);
}

void CsApexWindow::start()
{
    showStatusMessage("building ui");
    construct();
    showStatusMessage("initializing");
    init();
}

void CsApexWindow::updateMenu()
{
    ui->actionUndo->setDisabled(!cmd_dispatcher_->canUndo());
    ui->actionRedo->setDisabled(!cmd_dispatcher_->canRedo());
}

void CsApexWindow::updateTitle()
{
    std::stringstream window;
    window << "CS::APEX (" << core_.getSettings().getConfig() << ")";

    if(cmd_dispatcher_->isDirty()) {
        window << " *";
    }

    setWindowTitle(window.str().c_str());
}

void CsApexWindow::scrollDownLog()
{
    QTextCursor cursor = ui->logOutput->textCursor();
    cursor.movePosition(QTextCursor::End, QTextCursor::MoveAnchor);
    ui->logOutput->setTextCursor(cursor);
}

void CsApexWindow::tick()
{
    cmd_dispatcher_->executeLater();

    std::string latest_cout = StreamInterceptor::instance().getCout().c_str();
    std::string latest_cerr = StreamInterceptor::instance().getCerr().c_str();

//    if(!latest_cout.empty()) {
//        scrollDownLog();

//        std::stringstream latest;
//        latest << latest_cout;


//        std::string line;
//        while (std::getline(latest, line, '\n')) {
//            if(line.substr(0, 8) == "warning:") {
//                line = std::string("<span style='color: #ffcc00;'><b>") + line + "</b></span>";
//            }

//            line = BashParser::toHtml(line);

//            line += "<br />";
//            ui->logOutput->insertHtml(line.c_str());
//        }

//    }
//    if(!latest_cerr.empty()) {
//        size_t i = 0;
//        while((i = latest_cerr.find('\n', i)) != std::string::npos) {
//            latest_cerr.replace(i, 1, "<br />");
//            i += 6;
//        }

//        latest_cerr = std::string("<span style='color: red'><b>") + latest_cerr + "</b></span>";

//        scrollDownLog();

//        ui->logOutput->insertHtml(latest_cerr.c_str());

//        int height = 50;

//        QList<int> sizes = ui->splitter->sizes();
//        sizes[0] -= height;
//        sizes[1] = height;
//        ui->splitter->setSizes(sizes);
//    }
}

void CsApexWindow::hideLog()
{
    QList<int> sizes = ui->splitter->sizes();
    sizes[0] += sizes[1];
    sizes[1] = 0;
    ui->splitter->setSizes(sizes);
}

void CsApexWindow::closeEvent(QCloseEvent* event)
{
    if(cmd_dispatcher_->isDirty()) {
        int r = QMessageBox::warning(this, tr("cs::APEX"),
                                     tr("Do you want to save the layout before closing?"),
                                     QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        if(r == QMessageBox::Save) {
            std::cout << "save" << std::endl;

            save();
            event->accept();
        } else if(r == QMessageBox::Discard) {
            event->accept();
        } else {
            event->ignore();
            return;
        }
    }

    try {
        graph_->stop();
    } catch(...) {
        std::abort();
    }

    event->accept();
}

void CsApexWindow::showStatusMessage(const std::string &msg)
{
    Q_EMIT statusChanged(QString(msg.c_str()));
}

void CsApexWindow::reloadBoxMenues()
{
    designer_->reloadBoxMenues();
}

void CsApexWindow::init()
{
    init_ = true;

    designer_->show();
    hideLog();
}

void CsApexWindow::save()
{
    core_.saveAs(core_.getSettings().getConfig());
}

void CsApexWindow::saveAs()
{
    QString filename = QFileDialog::getSaveFileName(0, "Save config", core_.getSettings().getConfig().c_str(), Settings::config_selector.c_str());

    if(!filename.isEmpty()) {
        core_.saveAs(filename.toStdString());
        core_.getSettings().setCurrentConfig(filename.toStdString());
    }
}


void CsApexWindow::saveAsCopy()
{
    QString filename = QFileDialog::getSaveFileName(0, "Save config", core_.getSettings().getConfig().c_str(), Settings::config_selector.c_str());

    if(!filename.isEmpty()) {
        core_.saveAs(filename.toStdString());
    }
}

void CsApexWindow::reload()
{
    core_.load(core_.getSettings().getConfig());
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
    cmd_dispatcher_->execute(graph_->clear());
}

void CsApexWindow::undo()
{
    cmd_dispatcher_->undo();
}

void CsApexWindow::redo()
{
    cmd_dispatcher_->redo();
}

void CsApexWindow::load()
{
    QString filename = QFileDialog::getOpenFileName(0, "Load config", core_.getSettings().getConfig().c_str(), Settings::config_selector.c_str());

    if(QFile(filename).exists()) {
        core_.load(filename.toStdString());
    }
}

void CsApexWindow::saveSettings(YAML::Emitter &e)
{
    DesignerIO designerio(*designer_);
    designerio.saveSettings(e);
}

void CsApexWindow::loadSettings(YAML::Node &doc)
{
    DesignerIO designerio(*designer_);
    designerio.loadSettings(doc);
}


void CsApexWindow::openSubGraph(Group */*grp*/)
{
}
