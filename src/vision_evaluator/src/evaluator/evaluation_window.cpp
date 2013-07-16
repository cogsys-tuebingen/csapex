/// HEADER
#include "evaluation_window.h"

/// COMPONENT
#include "ui_evaluation_window.h"

/// PROJECT
#include <designer/box_manager.h>

/// SYSTEM
#include <iostream>
#include <opencv2/opencv.hpp>
#include <QCloseEvent>
#include <QObjectList>
#include <QSharedPointer>
#include <QMetaType>
#include <QMessageBox>

Q_DECLARE_METATYPE(cv::Mat)
Q_DECLARE_METATYPE(std::string)
Q_DECLARE_METATYPE(QSharedPointer<QImage>)

using namespace vision_evaluator;

EvaluationWindow::EvaluationWindow(const std::string& directory, QWidget* parent) :
    QMainWindow(parent), ui(new Ui::EvaluationWindow)
{
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<QSharedPointer<QImage> >("QSharedPointer<QImage>");

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
    window << "Vision Evaluator (" << ui->designer->getConfig() << ")";

    if(BoxManager::instance().isDirty()) {
        window << " *";
    }

    setWindowTitle(window.str().c_str());
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
}
