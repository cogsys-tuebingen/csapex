/// HEADER
#include <csapex/view/csapex_window.h>

/// COMPONENT
#include <csapex/core/designerio.h>
#include <csapex/core/graphio.h>
#include <csapex/core/settings.h>
#include <csapex/model/node_factory.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_worker.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_statistics.h>
#include <csapex/model/tag.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/view/box.h>
#include <csapex/view/designer.h>
#include <csapex/view/widget_controller.h>
#include "ui_csapex_window.h"

/// PROJECT
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <iostream>
#include <QCloseEvent>
#include <QMessageBox>
#include <QFileDialog>
#include <QApplication>
#include <QTreeWidget>

using namespace csapex;

CsApexWindow::CsApexWindow(CsApexCore& core, CommandDispatcher* cmd_dispatcher, WidgetControllerPtr widget_ctrl, GraphWorkerPtr graph, Designer* designer, QWidget *parent)
    : QMainWindow(parent), core_(core), cmd_dispatcher_(cmd_dispatcher), widget_ctrl_(widget_ctrl), graph_worker_(graph), ui(new Ui::CsApexWindow), designer_(designer), init_(false), style_sheet_watcher_(NULL)
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

    Graph* graph = graph_worker_->getGraph();

    designer_->setup();
    setCentralWidget(designer_);

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
    QObject::connect(ui->actionClearBlock, SIGNAL(triggered(bool)), this, SLOT(clearBlock()));

    QObject::connect(ui->actionGrid, SIGNAL(toggled(bool)), designer_,  SLOT(enableGrid(bool)));
    QObject::connect(designer_, SIGNAL(gridEnabled(bool)), ui->actionGrid, SLOT(setChecked(bool)));
    QObject::connect(ui->actionSchematics, SIGNAL(toggled(bool)), designer_,  SLOT(enableSchematics(bool)));
    QObject::connect(designer_, SIGNAL(schematicsEnabled(bool)), ui->actionSchematics, SLOT(setChecked(bool)));
    QObject::connect(ui->actionLock_to_Grid, SIGNAL(toggled(bool)), designer_,  SLOT(lockToGrid(bool)));
    QObject::connect(designer_, SIGNAL(gridLockEnabled(bool)), ui->actionLock_to_Grid, SLOT(setChecked(bool)));

    QObject::connect(ui->actionDelete_Selected, SIGNAL(triggered(bool)), designer_, SLOT(deleteSelected()));
    QObject::connect(designer_, SIGNAL(selectionChanged()), this, SLOT(updateDeleteAction()));
    QObject::connect(designer_, SIGNAL(selectionChanged()), this, SLOT(updateDebugInfo()));
    QObject::connect(designer_, SIGNAL(helpRequest(NodeBox*)), this, SLOT(showHelp(NodeBox*)));

    QObject::connect(ui->actionClear_selection, SIGNAL(triggered()), designer_,  SLOT(clearSelection()));
    QObject::connect(ui->actionSelect_all, SIGNAL(triggered()), designer_,  SLOT(selectAll()));

    QObject::connect(ui->actionAbout_CS_APEX, SIGNAL(triggered()), this, SLOT(about()));

    QObject::connect(ui->node_info_tree, SIGNAL(itemSelectionChanged()), this, SLOT(updateNodeInfo()));

    QObject::connect(graph, SIGNAL(stateChanged()), designer_, SLOT(stateChangedEvent()));
    QObject::connect(graph, SIGNAL(stateChanged()), this, SLOT(updateMenu()));

    QObject::connect(&core_, SIGNAL(configChanged()), this, SLOT(updateTitle()));
    QObject::connect(&core_, SIGNAL(showStatusMessage(const std::string&)), this, SLOT(showStatusMessage(const std::string&)));
    QObject::connect(&core_, SIGNAL(reloadBoxMenues()), this, SLOT(reloadBoxMenues()));

    QObject::connect(&core_, SIGNAL(resetRequest()), designer_, SLOT(reset()));

    QObject::connect(&core_, SIGNAL(saveSettingsRequest(YAML::Node&)), this, SLOT(saveSettings(YAML::Node&)));
    QObject::connect(&core_, SIGNAL(loadSettingsRequest(YAML::Node&)), this, SLOT(loadSettings(YAML::Node&)));
    QObject::connect(&core_, SIGNAL(saveViewRequest(YAML::Node&)), this, SLOT(saveView(YAML::Node&)));
    QObject::connect(&core_, SIGNAL(loadViewRequest(YAML::Node&)), this, SLOT(loadView(YAML::Node&)));

    QObject::connect(graph, SIGNAL(nodeAdded(NodePtr)), widget_ctrl_.get(), SLOT(nodeAdded(NodePtr)));
    QObject::connect(graph, SIGNAL(nodeRemoved(NodePtr)), widget_ctrl_.get(), SLOT(nodeRemoved(NodePtr)));

    QObject::connect(graph, SIGNAL(dirtyChanged(bool)), this, SLOT(updateTitle()));

    QObject::connect(cmd_dispatcher_, SIGNAL(stateChanged()), this, SLOT(updateUndoInfo()));

    updateMenu();
    updateTitle();

    timer.setInterval(100);
    timer.setSingleShot(false);
    timer.start();

    QObject::connect(&timer, SIGNAL(timeout()), this, SLOT(tick()));
}

void CsApexWindow::updateDeleteAction()
{
    ui->actionDelete_Selected->setEnabled(designer_->hasSelection());
}

void CsApexWindow::showHelp(NodeBox *box)
{
    if(ui->HelpCenter->isHidden()) {
        ui->HelpCenter->show();
    }


    std::string node_type = box->getNode()->getType();

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

void CsApexWindow::updateDebugInfo()
{
    if(!ui->Debug->isVisible()) {
        return;
    }

    ui->box_info->clear();

    std::vector<NodeBox*> selected = designer_->getSelectedBoxes();

    foreach (NodeBox* box, selected) {
        Node* node = box->getNode();
        NodeWorker* worker = node->getNodeWorker();
        QObject::connect(worker, SIGNAL(nodeStateChanged()), this, SLOT(updateDebugInfo()));
        QObject::connect(worker, SIGNAL(nodeModelChanged()), this, SLOT(updateDebugInfo()));
        ui->box_info->addTopLevelItem(NodeStatistics(node).createDebugInformation(widget_ctrl_->getNodeFactory()));
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

    Q_FOREACH(QTreeWidgetItem* item, ui->node_info_tree->selectedItems()) {
        QString type = item->data(0, Qt::UserRole + 1).toString();
        if(!type.isEmpty()) {
            NodeConstructor::Ptr n = widget_ctrl_->getNodeFactory()->getConstructor(type.toStdString());

            QImage image = n->getIcon().pixmap(QSize(16,16)).toImage();
            QUrl uri ( QString::fromStdString(n->getType()));
            ui->node_info_text->document()->addResource( QTextDocument::ImageResource, uri, QVariant ( image ) );

            ss << "<h1> <img src=\"" << uri.toString().toStdString() << "\" /> " << n->getType() << "</h1>";
            ss << "<p>Tags: <i>";
            int i = 0;
            Q_FOREACH(const Tag::Ptr& tag, n->getTags()) {
                if(i++ > 0) {
                    ss << ", ";
                }
                ss << tag->getName();
            }
            ss << "</p>";
            ss << "<h3>" << n->getDescription() << "</h3>";

            ss << "<hr />";
            ss << "<h1>Parameters:</h1>";

            std::vector<param::Parameter::Ptr> params = n->makePrototypeContent()->getParameters();

            Q_FOREACH(const param::Parameter::Ptr& p, params) {
                ss << "<h2>" << p->name() << "</h2>";
                ss << "<p>" << p->description().toString() << "</p>";
                ss << "<p>" << p->toString() << "</p>";
            }
        }
    }


    ui->node_info_text->setHtml(QString::fromStdString(ss.str()));
}

void CsApexWindow::updateUndoInfo()
{
    ui->undo->clear();
    ui->redo->clear();

    cmd_dispatcher_->populateDebugInfo(ui->undo, ui->redo);

    ui->undo->expandAll();
    ui->redo->expandAll();
}

void CsApexWindow::about()
{
    std::stringstream ss;
    ss << "<h1>cs::APEX 0.9</h1>";
    ss << "<p>Based on QT " << QT_VERSION_STR ;
#ifdef __GNUC__
    ss << " (GCC " << __VERSION__ << ")";
#endif
    ss << "</p>";
    ss << "<p>Built on " << __DATE__ << " at " << __TIME__ << "</p>";

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

    ss << "<p>From revision " << TOSTRING(GIT_COMMIT_HASH) << " (" << TOSTRING(GIT_BRANCH) << ")</p>";
    ss << "<p>The program is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.</p>";

    QMessageBox::about(this, "About cs::APEX", ss.str().c_str());
}

void CsApexWindow::clearBlock()
{
    QMessageBox::information(this, "Not yet implemented", "Sorry, this feature is not implemented yet.");
}

void CsApexWindow::reloadBoxMenues()
{
    if(ui->boxes->layout()) {
        QtHelper::clearLayout(ui->boxes->layout());
    } else {
        ui->boxes->setLayout(new QVBoxLayout);
    }
    if(ui->node_info_tree->layout()) {
        QtHelper::clearLayout(ui->node_info_tree->layout());
    } else {
        ui->node_info_tree->setLayout(new QVBoxLayout);
    }

    widget_ctrl_->insertAvailableNodeTypes(ui->boxes);
    widget_ctrl_->insertAvailableNodeTypes(ui->node_info_tree);
}


void CsApexWindow::resetSignal()
{
    designer_->reset();
}

void CsApexWindow::loadStyleSheet(const QString& path)
{
    QFile file(path);
    file.open(QFile::ReadOnly);
    QString style_sheet(file.readAll());
    QWidget::setStyleSheet(style_sheet);
    widget_ctrl_->setStyleSheet(style_sheet);

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
    ui->actionUndo->setDisabled(!cmd_dispatcher_->canUndo());
    ui->actionRedo->setDisabled(!cmd_dispatcher_->canRedo());
}

void CsApexWindow::updateTitle()
{
    std::stringstream window;
    window << "CS::APEX (" << getConfigFile() << ")";

    if(cmd_dispatcher_->isDirty()) {
        window << " *";
    }

    setWindowTitle(window.str().c_str());
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

    QString geometry(saveGeometry().toBase64());
    QString uistate(saveState().toBase64());

    Settings& settings = core_.getSettings();
    if(!settings.knows("uistate")) {
        settings.add(param::ParameterFactory::declareText("uistate", ""));
    }
    if(!settings.knows("geometry")) {
        settings.add(param::ParameterFactory::declareText("geometry", "geometry.toStdString()"));
    }

    settings.set("uistate", uistate.toStdString());
    settings.set("geometry", geometry.toStdString());
    core_.settingsChanged();

    try {
        graph_worker_->stop();
    } catch(...) {
        std::abort();
    }

    event->accept();
}

void CsApexWindow::showStatusMessage(const std::string &msg)
{
    Q_EMIT statusChanged(QString(msg.c_str()));
}

void CsApexWindow::init()
{
    init_ = true;

    reloadBoxMenues();
    //    designer_->show();

    Settings& settings = core_.getSettings();
    if(settings.knows("uistate")) {
        std::string uistate = settings.get<std::string>("uistate");
        restoreState(QByteArray::fromBase64(uistate.data()));
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
    QString filename = QFileDialog::getSaveFileName(0, "Save config", QString::fromStdString(getConfigFile()), QString::fromStdString(Settings::config_selector));

    if(!filename.isEmpty()) {
        core_.saveAs(filename.toStdString());
        core_.getSettings().set("config", filename.toStdString());
    }
}


void CsApexWindow::saveAsCopy()
{
    QString filename = QFileDialog::getSaveFileName(0, "Save config", QString::fromStdString(getConfigFile()), QString::fromStdString(Settings::config_selector));

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
    cmd_dispatcher_->execute(graph_worker_->getGraph()->clear());
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
    QString filename = QFileDialog::getOpenFileName(0, "Load config", QString::fromStdString(getConfigFile()), QString::fromStdString(Settings::config_selector));

    if(QFile(filename).exists()) {
        core_.load(filename.toStdString());
    }
}

void CsApexWindow::saveSettings(YAML::Node& doc)
{
    DesignerIO designerio(*designer_);
    designerio.saveSettings(doc);
}

void CsApexWindow::loadSettings(YAML::Node &doc)
{
    DesignerIO designerio(*designer_);
    designerio.loadSettings(doc);
}


void CsApexWindow::saveView(YAML::Node &doc)
{
    DesignerIO designerio(*designer_);
    designerio.saveBoxes(doc, graph_worker_->getGraph(), widget_ctrl_.get());
}

void CsApexWindow::loadView(YAML::Node &doc)
{
    DesignerIO designerio(*designer_);
    designerio.loadBoxes(doc, widget_ctrl_.get());
}

