/// HEADER
#include "evaluation_window.h"

/// COMPONENT
#include "ui_evaluation_window.h"
#include "image_panel.h"

/// SYSTEM
#include <iostream>
#include <QCloseEvent>
#include <QObjectList>
#include <QSharedPointer>
#include <QMetaType>

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

    QList<ImagePanel*> ips = findChildren<ImagePanel*>(QRegExp(".*"));
    for(QList<ImagePanel*>::Iterator it = ips.begin(); it != ips.end(); ++it) {
        (*it)->set_root(directory);
    }

    QObject::connect(ui->left, SIGNAL(outputMat(cv::Mat, cv::Mat)), ui->combiner, SLOT(input_1(cv::Mat, cv::Mat)));
    QObject::connect(ui->right, SIGNAL(outputMat(cv::Mat, cv::Mat)), ui->combiner, SLOT(input_2(cv::Mat, cv::Mat)));

    QObject::connect(ui->combiner, SIGNAL(nextImageRequest()), ui->left, SLOT(nextImage()));
    QObject::connect(ui->combiner, SIGNAL(nextImageRequest()), ui->right, SLOT(nextImage()));

    QObject::connect(ui->combiner, SIGNAL(combinerInstalled()), ui->left, SLOT(setOneShotModeOn()));
    QObject::connect(ui->combiner, SIGNAL(combinerInstalled()), ui->right, SLOT(setOneShotModeOn()));
    QObject::connect(ui->combiner, SIGNAL(combinerDeinstalled()), ui->left, SLOT(setOneShotModeOff()));
    QObject::connect(ui->combiner, SIGNAL(combinerDeinstalled()), ui->right, SLOT(setOneShotModeOff()));

    QObject::connect(ui->fps, SIGNAL(valueChanged(int)), this, SLOT(set_fps(int)));
}

void EvaluationWindow::set_fps(int fps)
{
    QList<ImagePanel*> ips = findChildren<ImagePanel*>(QRegExp(".*"));
    for(QList<ImagePanel*>::Iterator it = ips.begin(); it != ips.end(); ++it) {
        (*it)->set_fps(fps);
    }
}

void EvaluationWindow::closeEvent(QCloseEvent* event)
{
    ui->left->quit();
    ui->right->quit();
    ui->solo_image->quit();
    ui->combiner->quit();

    ui->left->wait();
    ui->right->wait();
    ui->solo_image->wait();
    ui->combiner->wait();

    event->accept();
}

void EvaluationWindow::setSecondaryDirectory(const std::string& directory)
{
    ui->right->set_root(directory);
}

void EvaluationWindow::setSingleMode()
{
    ui->mode_tab->setCurrentIndex(1);
}

void EvaluationWindow::setDualMode()
{
    ui->mode_tab->setCurrentIndex(1);
}
