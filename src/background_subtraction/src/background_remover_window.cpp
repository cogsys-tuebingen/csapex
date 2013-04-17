/// HEADER
#include "background_remover_window.h"

/// COMPONENT
#include "ui_background_remover_window.h"

/// SYSTEM
#include <dynamic_reconfigure/server.h>
#include <QComboBox>
#include <QPushButton>
#include <QPixmap>
#include <QSlider>

using namespace background_subtraction;

BackgroundRemoverWindow::BackgroundRemoverWindow(ros::NodeHandle& nh, QWidget* parent) :
    QMainWindow(parent),
    ui(new Ui::BackgroundRemoverWindow), node(nh, img_mutex)
{
    ui->setupUi(this);

    lt = new QGraphicsScene();
    rt = new QGraphicsScene();
    lb = new QGraphicsScene();
    rb = new QGraphicsScene();
    t = new QGraphicsScene();

    ui->left_top->setScene(lt);
    ui->right_top->setScene(rt);
    ui->left_bottom->setScene(lb);
    ui->right_bottom->setScene(rb);
    ui->top->setScene(t);

    QObject::connect(&node, SIGNAL(imageChanged()), this, SLOT(updateImage()));
    QObject::connect(&node, SIGNAL(configChanged()), this, SLOT(updateUi()));
    QObject::connect(ui->set_bg_btn, SIGNAL(clicked()), &node, SLOT(setBackground()));
    QObject::connect(ui->algorithm, SIGNAL(currentIndexChanged(int)), this, SLOT(changeAlgo(int)));

    QObject::connect(ui->threshold, SIGNAL(sliderMoved(int)), this, SLOT(setThreshold(int)));
    QObject::connect(ui->open, SIGNAL(sliderMoved(int)), this, SLOT(setOpen(int)));
    QObject::connect(ui->close, SIGNAL(sliderMoved(int)), this, SLOT(setClose(int)));

    node.init();

    std::vector<BackgroundRemover*>& removers = BackgroundRemover::metaInstance().instances;
    for(std::vector<BackgroundRemover*>::iterator it = removers.begin(); it != removers.end(); ++it) {
        ui->algorithm->addItem(tr((*it)->getName().c_str()));
    }

    ui->threshold->setValue(node.active_background_remover->getThreshold());
    ui->open->setValue(node.active_background_remover->getOpen());
    ui->close->setValue(node.active_background_remover->getClose());
}

BackgroundRemoverWindow::~BackgroundRemoverWindow()
{
    delete ui;
}

void BackgroundRemoverWindow::shutdown()
{
    ros::shutdown();
}

void BackgroundRemoverWindow::changeAlgo(int i)
{
    background_subtraction::GlobalConfig cfg = node.getCurrentConfig();
    cfg.algorithm = i;
    node.update(cfg);
}

void BackgroundRemoverWindow::setThreshold(int i)
{
    background_subtraction::GlobalConfig cfg = node.getCurrentConfig();
    cfg.threshold = i;
    node.update(cfg);
}

void BackgroundRemoverWindow::setClose(int i)
{
    background_subtraction::GlobalConfig cfg = node.getCurrentConfig();
    cfg.close = i;
    node.update(cfg);
}

void BackgroundRemoverWindow::setOpen(int i)
{
    background_subtraction::GlobalConfig cfg = node.getCurrentConfig();
    cfg.open = i;
    node.update(cfg);
}

void BackgroundRemoverWindow::updateUi()
{
    background_subtraction::GlobalConfig cfg = node.getCurrentConfig();

    ui->algorithm->setCurrentIndex(cfg.algorithm);
    ui->threshold->setSliderPosition(cfg.threshold);
    ui->open->setSliderPosition(cfg.open);
    ui->close->setSliderPosition(cfg.close);
}

void BackgroundRemoverWindow::updateImage()
{
    img_mutex.lock();

    if(node.has_images) {
        lt->clear();
        lt->addPixmap(QPixmap::fromImage(*node.frame_));
        rt->clear();
        rt->addPixmap(QPixmap::fromImage(*node.bg_image_));
        lb->clear();
        lb->addPixmap(QPixmap::fromImage(*node.filtered_));
        t->clear();
        t->addPixmap(QPixmap::fromImage(*node.debug_));
        rb->clear();
        rb->addPixmap(QPixmap::fromImage(*node.mask_));
    }

    img_mutex.unlock();

    resize(sizeHint());
}
