/// HEADER
#include <csapex/csapex_window.h>

/// COMPONENT
#include "ui_evaluation_window.h"
#include <csapex/bash_parser.h>
#include <csapex/core_plugin.h>
#include <csapex/box_manager.h>
#include <csapex/command_dispatcher.h>
#include <csapex/designer.h>
#include <csapex/graph.h>
#include <csapex/stream_interceptor.h>
#include <csapex/qt_helper.hpp>
#include <csapex/designerio.h>
#include <csapex/graphio.h>
#include <csapex/tag.h>

/// SYSTEM
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <QCloseEvent>
#include <QObjectList>
#include <QSharedPointer>
#include <QMessageBox>
#include <QStatusBar>
#include <QToolBar>
#include <QTimer>
#include <QFileDialog>

using namespace csapex;

CsApexWindow::CsApexWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::EvaluationWindow), core_plugin_manager("csapex::CorePlugin"), init_(false)
{
    Graph::Ptr graph = Graph::root();

    StreamInterceptor::instance().start();

    ui->setupUi(this);

    designer_ = new Designer;
    designer_->hide();
    ui->splitter->addWidget(designer_);
    ui->splitter->addWidget(ui->logOutput);

    Tag::createIfNotExists("General");

    QObject::connect(ui->actionSave, SIGNAL(triggered()), this, SLOT(save()));
    QObject::connect(ui->actionSaveAs, SIGNAL(triggered()), this,  SLOT(saveAs()));
    QObject::connect(ui->actionLoad, SIGNAL(triggered()), this,  SLOT(load()));
    QObject::connect(ui->actionReload, SIGNAL(triggered()), this,  SLOT(reload()));
    QObject::connect(ui->actionUndo, SIGNAL(triggered()), graph.get(),  SLOT(undo()));
    QObject::connect(ui->actionRedo, SIGNAL(triggered()), graph.get(),  SLOT(redo()));
    QObject::connect(ui->actionClear, SIGNAL(triggered()), graph.get(),  SLOT(clear()));

    QObject::connect(graph.get(), SIGNAL(boxAdded(Box*)), designer_, SLOT(addBox(Box*)));
    QObject::connect(graph.get(), SIGNAL(boxDeleted(Box*)), designer_, SLOT(deleteBox(Box*)));
    QObject::connect(graph.get(), SIGNAL(stateChanged()), designer_, SLOT(stateChangedEvent()));
    QObject::connect(graph.get(), SIGNAL(stateChanged()), this, SLOT(updateMenu()));

    QObject::connect(this, SIGNAL(configChanged()), this, SLOT(updateTitle()));
    QObject::connect(&CommandDispatcher::instance(), SIGNAL(dirtyChanged(bool)), this, SLOT(updateTitle()));

    QObject::connect(this, SIGNAL(initialize()), this, SLOT(init()), Qt::QueuedConnection);

    updateMenu();
    updateTitle();

    hideLog();

    timer.setInterval(100);
    timer.setSingleShot(false);
    timer.start();

    setCurrentConfig(GraphIO::default_config);

    QObject::connect(&timer, SIGNAL(timeout()), this, SLOT(tick()));
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
    statusBar()->showMessage("initialized");

    ui->splitter->hide();

    resize(250,120);

    show();
}

void CsApexWindow::updateMenu()
{
    Graph::Ptr graph_ = Graph::root();
    ui->actionUndo->setDisabled(!graph_->canUndo());
    ui->actionRedo->setDisabled(!graph_->canRedo());
}

void CsApexWindow::updateTitle()
{
    std::stringstream window;
    window << "CS::APEX (" << getConfig() << ")";

    if(CommandDispatcher::instance().isDirty()) {
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
    CommandDispatcher::executeLater();

    std::string latest_cout = StreamInterceptor::instance().getCout().c_str();
    std::string latest_cerr = StreamInterceptor::instance().getCerr().c_str();

    if(!latest_cout.empty()) {
        scrollDownLog();

        std::stringstream latest;
        latest << latest_cout;


        std::string line;
        while (std::getline(latest, line, '\n')) {
            if(line.substr(0, 8) == "warning:") {
                line = std::string("<span style='color: #ffcc00;'><b>") + line + "</b></span>";
            }

            line = BashParser::toHtml(line);

            line += "<br />";
            ui->logOutput->insertHtml(line.c_str());
        }

    }
    if(!latest_cerr.empty()) {
        size_t i = 0;
        while((i = latest_cerr.find('\n', i)) != std::string::npos) {
            latest_cerr.replace(i, 1, "<br />");
            i += 6;
        }

        latest_cerr = std::string("<span style='color: red'><b>") + latest_cerr + "</b></span>";

        scrollDownLog();

        ui->logOutput->insertHtml(latest_cerr.c_str());

        int height = 50;

        QList<int> sizes = ui->splitter->sizes();
        sizes[0] -= height;
        sizes[1] = height;
        ui->splitter->setSizes(sizes);
    }
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
    Graph::Ptr graph_ = Graph::root();
    if(graph_->isDirty()) {
        int r = QMessageBox::warning(this, tr("Vision Designer"),
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

    StreamInterceptor::instance().stop();
    BoxManager::instance().stop();
}

void CsApexWindow::showStatusMessage(const std::string &msg)
{
    statusBar()->showMessage(msg.c_str());
}

void CsApexWindow::init()
{
    if(!init_) {
        init_ = true;

        statusBar()->showMessage("loading core plugins");

        core_plugin_manager.reload();

        typedef const std::pair<std::string, PluginManager<CorePlugin>::Constructor> PAIR;
        foreach(PAIR cp, core_plugin_manager.availableClasses()) {
            CorePlugin::Ptr plugin = cp.second();

            plugin->init();
        }

        statusBar()->showMessage("loading boxedobject plugins");

        BoxManager& bm = BoxManager::instance();

        bm.loaded.connect(boost::bind(&CsApexWindow::showStatusMessage, this, _1));

        bm.reload();

        statusBar()->showMessage("loading config");

        resize(400,400);

        reload();

        statusBar()->hide();
        ui->loading->hide();
        repaint();
        designer_->show();
        ui->splitter->show();
        hideLog();
    }
}

void CsApexWindow::paintEvent(QPaintEvent *e)
{
    QMainWindow::paintEvent(e);

    if(!init_) {
        Q_EMIT initialize();
    }
}

void CsApexWindow::save()
{
    saveAs(current_config_);
}

void CsApexWindow::setCurrentConfig(const std::string& filename)
{
    current_config_ = filename;

    std::string dir = current_config_.substr(0, current_config_.find_last_of('/')+1);
    chdir(dir.c_str());

    Q_EMIT configChanged();
}

std::string CsApexWindow::getConfig() const
{
    return current_config_;
}

void CsApexWindow::saveAs()
{
    QString filename = QFileDialog::getSaveFileName(0, "Save config", current_config_.c_str(), GraphIO::config_selector.c_str());

    if(!filename.isEmpty()) {
        saveAs(filename.toStdString());
        setCurrentConfig(filename.toStdString());
    }
}

void CsApexWindow::load()
{
    QString filename = QFileDialog::getOpenFileName(0, "Load config", current_config_.c_str(), GraphIO::config_selector.c_str());

    if(QFile(filename).exists()) {
        setCurrentConfig(filename.toStdString());

        reload();
    }
}

void CsApexWindow::saveAs(const std::string &file)
{
    std::string dir = file.substr(0, file.find_last_of('/')+1);
    chdir(dir.c_str());

    YAML::Emitter yaml;

    yaml << YAML::BeginMap; // settings map

    GraphIO graphio(Graph::root());
    DesignerIO designerio(*designer_);

    designerio.saveSettings(yaml);
    graphio.saveSettings(yaml);
    graphio.saveConnections(yaml);

    yaml << YAML::EndMap; // settings map

    graphio.saveBoxes(yaml);

    std::ofstream ofs(file.c_str());
    ofs << yaml.c_str();

    std::cout << "save: " << yaml.c_str() << std::endl;

    CommandDispatcher::instance().setClean();
    CommandDispatcher::instance().resetDirtyPoint();
}


void CsApexWindow::reload()
{
    Graph::Ptr graph_ = Graph::root();
    graph_->clear();

    GraphIO graphio(graph_);
    DesignerIO designerio(*designer_);

    {
        std::ifstream ifs(current_config_.c_str());
        YAML::Parser parser(ifs);

        YAML::Node doc;

        if(!parser.GetNextDocument(doc)) {
            std::cerr << "cannot read the config" << std::endl;
            return;
        }

        designerio.loadSettings(doc);
        graphio.loadSettings(doc);

        graphio.loadBoxes(parser);
    }
    {
        std::ifstream ifs(current_config_.c_str());
        YAML::Parser parser(ifs);

        YAML::Node doc;

        if(!parser.GetNextDocument(doc)) {
            std::cerr << "cannot read the config" << std::endl;
            return;
        }
        graphio.loadConnections(doc);
    }

    CommandDispatcher::instance().setClean();
    CommandDispatcher::instance().resetDirtyPoint();
}
