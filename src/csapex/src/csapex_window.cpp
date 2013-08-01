/// HEADER
#include <csapex/csapex_window.h>

/// COMPONENT
#include "ui_evaluation_window.h"
#include "bash_parser.h"
#include <csapex/core_plugin.h>
#include <csapex/box_manager.h>
#include <csapex/stream_interceptor.h>
#include <csapex/qt_helper.hpp>

/// SYSTEM
#include <iostream>
#include <opencv2/opencv.hpp>
#include <QCloseEvent>
#include <QObjectList>
#include <QSharedPointer>
#include <QMessageBox>
#include <QToolBar>
#include <QTimer>

using namespace csapex;

EvaluationWindow::EvaluationWindow(QWidget* parent) :
    QMainWindow(parent), ui(new Ui::EvaluationWindow)
{
    PluginManager<CorePlugin> core("csapex::CorePlugin");
    core.reload();

    typedef const std::pair<std::string, PluginManager<CorePlugin>::Constructor> PAIR;
    foreach(PAIR cp, core.availableClasses()) {
        CorePlugin::Ptr plugin = cp.second();
        plugin->init();
    }

    ui->setupUi(this);

    QObject::connect(ui->actionSave, SIGNAL(triggered()), ui->designer, SLOT(save()));
    QObject::connect(ui->actionSaveAs, SIGNAL(triggered()), ui->designer, SLOT(saveAs()));
    QObject::connect(ui->actionLoad, SIGNAL(triggered()), ui->designer, SLOT(load()));
    QObject::connect(ui->actionReload, SIGNAL(triggered()), ui->designer, SLOT(reload()));
    QObject::connect(ui->actionUndo, SIGNAL(triggered()), ui->designer, SLOT(undo()));
    QObject::connect(ui->actionRedo, SIGNAL(triggered()), ui->designer, SLOT(redo()));
    QObject::connect(ui->actionClear, SIGNAL(triggered()), ui->designer, SLOT(clear()));

    QObject::connect(ui->designer, SIGNAL(stateChanged()), this, SLOT(updateMenu()));

    QObject::connect(ui->designer, SIGNAL(configChanged()), this, SLOT(updateTitle()));
    QObject::connect(&BoxManager::instance(), SIGNAL(dirtyChanged(bool)), this, SLOT(updateTitle()));

    updateMenu();
    updateTitle();

    hideLog();

    timer.setInterval(100);
    timer.setSingleShot(false);
    timer.start();

    QObject::connect(&timer, SIGNAL(timeout()), this, SLOT(updateLog()));
}

Designer* EvaluationWindow::getDesigner()
{
    return ui->designer;
}

void EvaluationWindow::showMenu()
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

void EvaluationWindow::start()
{
    show();
    ui->designer->reload();
}

void EvaluationWindow::updateMenu()
{
    ui->actionUndo->setDisabled(!ui->designer->canUndo());
    ui->actionRedo->setDisabled(!ui->designer->canRedo());
}

void EvaluationWindow::updateTitle()
{
    std::stringstream window;
    window << "CS::APEX (" << ui->designer->getConfig() << ")";

    if(BoxManager::instance().isDirty()) {
        window << " *";
    }

    setWindowTitle(window.str().c_str());
}

void EvaluationWindow::scrollDownLog()
{
    QTextCursor cursor = ui->logOutput->textCursor();
    cursor.movePosition(QTextCursor::End, QTextCursor::MoveAnchor);
    ui->logOutput->setTextCursor(cursor);
}

void EvaluationWindow::updateLog()
{
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

void EvaluationWindow::hideLog()
{
    QList<int> sizes = ui->splitter->sizes();
    sizes[0] += sizes[1];
    sizes[1] = 0;
    ui->splitter->setSizes(sizes);
}

void EvaluationWindow::closeEvent(QCloseEvent* event)
{
    if(ui->designer->isDirty()) {
        int r = QMessageBox::warning(this, tr("Vision Designer"),
                                     tr("Do you want to save the layout before closing?"),
                                     QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        if(r == QMessageBox::Save) {
            std::cout << "save" << std::endl;

            ui->designer->save();
            event->accept();
        } else if(r == QMessageBox::Discard) {
            event->accept();
        } else {
            event->ignore();
            return;
        }
    }

    event->accept();

    StreamInterceptor::instance().stop();
}
