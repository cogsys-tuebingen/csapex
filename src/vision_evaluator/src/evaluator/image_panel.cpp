/// HEADER
#include "image_panel.h"

/// COMPONENT
#include "ui_image_panel.h"

/// SYSTEM
#include <boost/assign.hpp>
#include <iostream>
#include <QFileSystemModel>
#include <QHBoxLayout>

using namespace vision_evaluator;

ImagePanel::ImagePanel(QWidget* parent) :
    Panel(parent), ui(new Ui::ImagePanel), splitter_width_(0)
{
    ui->setupUi(this);

    model = new QFileSystemModel();
    model->setRootPath(tr("/"));
    ui->browser->setModel(model);

    QVBoxLayout* filter_layout = new QVBoxLayout;
    filter_manager.insert(filter_layout);
    filter_layout->addSpacerItem(new QSpacerItem(0, 1000, QSizePolicy::Minimum, QSizePolicy::Expanding));
    ui->filter_page->setLayout(filter_layout);

    QVBoxLayout* options_layout = new QVBoxLayout;
    option_manager.insert(options_layout);
    options_layout->addSpacerItem(new QSpacerItem(0, 1000, QSizePolicy::Minimum, QSizePolicy::Expanding));
    ui->options_page->setLayout(options_layout);

    view = ui->graphicsView;
    view->setScene(scene);

    viewer = new Viewer(ui->additional_frame);
    viewer->moveToThread(worker);


    QObject::connect(viewer, SIGNAL(image_provided(cv::Mat, cv::Mat)), &filter_manager, SLOT(filter(cv::Mat, cv::Mat)), Qt::DirectConnection);
    QObject::connect(this, SIGNAL(handle(std::string)), viewer, SLOT(handle(std::string)), Qt::QueuedConnection);

    QObject::connect(viewer, SIGNAL(update_request()), this, SLOT(update()), Qt::QueuedConnection);
    QObject::connect(this, SIGNAL(updateRequest()), viewer, SLOT(update_gui()), Qt::DirectConnection);

    QObject::connect(viewer, SIGNAL(gui_updated()), viewer, SLOT(tick()), Qt::QueuedConnection);

    QObject::connect(&filter_manager, SIGNAL(display_request(const QSharedPointer<QImage>)), this, SLOT(display_request(const QSharedPointer<QImage>)), Qt::DirectConnection);
    QObject::connect(&filter_manager, SIGNAL(outputMat(cv::Mat, cv::Mat)), this, SIGNAL(outputMat(cv::Mat, cv::Mat)), Qt::DirectConnection);

    QObject::connect(this, SIGNAL(display_request_gui(const QSharedPointer<QImage>)), this, SLOT(display(const QSharedPointer<QImage>)), Qt::QueuedConnection);

    QObject::connect(ui->browser, SIGNAL(activated(QModelIndex)), this, SLOT(set_root(QModelIndex)), Qt::QueuedConnection);
    QObject::connect(ui->browser->selectionModel(), SIGNAL(currentChanged(QModelIndex,QModelIndex)), this, SLOT(select(QModelIndex)), Qt::QueuedConnection);
    QObject::connect(ui->up_btn, SIGNAL(clicked()), this, SLOT(go_up()));
    QObject::connect(ui->go_button, SIGNAL(clicked()), this, SLOT(go_location()));
    QObject::connect(ui->go_text, SIGNAL(returnPressed()), this, SLOT(go_location()));

    QObject::connect(ui->hide_browser, SIGNAL(clicked()), this, SLOT(minimize_splitter()));
    QObject::connect(ui->show_browser, SIGNAL(clicked()), this, SLOT(restore_splitter()));

    set_root(QDir::currentPath().toUtf8().constData());
}

ImagePanel::~ImagePanel()
{
    delete ui;
    delete model;
    delete viewer;
}

void ImagePanel::setOneShotModeOn()
{
    viewer->setOneShotMode(true);
}

void ImagePanel::setOneShotModeOff()
{
    viewer->setOneShotMode(false);
}

void ImagePanel::nextImage()
{
    viewer->provideNextImage();
}

void ImagePanel::quit()
{
    worker->quit();
}

void ImagePanel::wait()
{
    worker->wait();
}

void ImagePanel::minimize_splitter()
{
    QList<int> sizes = ui->splitter->sizes();

    splitter_width_ = sizes[0];
    sizes[0] = 0;
    sizes[1] += splitter_width_;
    ui->splitter->setSizes(sizes);
}


void ImagePanel::restore_splitter()
{
    QList<int> sizes = ui->splitter->sizes();

    sizes[0] = splitter_width_;
    sizes[1] -= splitter_width_;
    ui->splitter->setSizes(sizes);
}

void ImagePanel::set_root(const std::string& path) const
{
    set_root(model->index(tr(path.c_str())));
}

void ImagePanel::set_fps(int fps)
{
    viewer->set_fps(fps);
}

void ImagePanel::set_root(QModelIndex index) const
{
    if(model->isDir(index)) {
        ui->browser->setRootIndex(index);
        ui->browser->setCurrentIndex(index);
        ui->go_text->setText(model->filePath(index));
    }
}

void ImagePanel::select(QModelIndex index)
{
    if(model->isDir(index) && (ui->browser->rootIndex() == index)) {
        return;
    }

    Q_EMIT handle(model->filePath(index).toUtf8().constData());
}

void ImagePanel::update()
{
    Q_EMIT updateRequest();
}

void ImagePanel::go_up()
{
    QModelIndex current = ui->browser->currentIndex();
    QModelIndex new_index = model->parent(current);

    if(new_index.isValid()) {
        set_root(new_index);
    }
}

void ImagePanel::go_location()
{
    QModelIndex index = model->index(ui->go_text->text());
    if(index.isValid()) {
        ui->browser->setRootIndex(index);
    }
}
