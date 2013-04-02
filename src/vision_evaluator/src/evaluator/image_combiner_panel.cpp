/// HEADER
#include "image_combiner_panel.h"

/// COMPONENT
#include "ui_image_combiner_panel.h"
#include "image_combiner.h"

/// SYSTEM
#include <boost/assign.hpp>
#include <QBoxLayout>
#include <QMouseEvent>
#include <QGraphicsSceneMouseEvent>

using namespace vision_evaluator;

ImageCombinerPanel::ImageCombinerPanel(QWidget* parent) :
    Panel(parent), ui(new Ui::ImageCombinerPanel), has_1(false), has_2(false)
{
    ui->setupUi(this);

    view = ui->view;
    view->setScene(scene);


    QHBoxLayout* combiner_layout = new QHBoxLayout;
    combiner_manager.insert(combiner_layout);
//    combiner_layout->addSpacerItem(new QSpacerItem(0, 1000, QSizePolicy::Minimum, QSizePolicy::Expanding));
    ui->menu->setLayout(combiner_layout);


    QObject::connect(this, SIGNAL(display_request_gui(QSharedPointer<QImage>)), this, SLOT(display(QSharedPointer<QImage>)));
    QObject::connect(&combiner_manager, SIGNAL(display_request(QSharedPointer<QImage>)), this, SLOT(display_request(QSharedPointer<QImage>)), Qt::QueuedConnection);
    QObject::connect(this, SIGNAL(handle(cv::Mat, cv::Mat, cv::Mat, cv::Mat)), &combiner_manager, SLOT(combine(cv::Mat, cv::Mat, cv::Mat, cv::Mat)), Qt::QueuedConnection);

    QObject::connect(&combiner_manager, SIGNAL(combinerInstalled()), this, SIGNAL(combinerInstalled()));
    QObject::connect(&combiner_manager, SIGNAL(combinerDeinstalled()), this, SIGNAL(combinerDeinstalled()));
    QObject::connect(&combiner_manager, SIGNAL(nextImageRequest()), this, SIGNAL(nextImageRequest()));


    view->setMouseTracking(true);
    scene->installEventFilter(this);
}

bool ImageCombinerPanel::eventFilter(QObject* target, QEvent* event)
{
    if(event->type() == QEvent::GraphicsSceneMouseMove) {
        QGraphicsSceneMouseEvent* ev = dynamic_cast<QGraphicsSceneMouseEvent*>(event);
        QMouseEvent mouseEv(QEvent::MouseMove, ev->screenPos(), ev->button(), ev->buttons(), ev->modifiers());
        combiner_manager.mouseMoveEvent(&mouseEv);
        return true;
    } else if(event->type() == QEvent::GraphicsSceneMousePress) {
        QGraphicsSceneMouseEvent* ev = dynamic_cast<QGraphicsSceneMouseEvent*>(event);
        QMouseEvent mouseEv(QEvent::MouseButtonPress, ev->screenPos(), ev->button(), ev->buttons(), ev->modifiers());
        combiner_manager.mousePressEvent(&mouseEv);
        return true;
    } else if(event->type() == QEvent::GraphicsSceneWheel) {
        QGraphicsSceneWheelEvent* ev = dynamic_cast<QGraphicsSceneWheelEvent*>(event);
        QWheelEvent wheelEv(ev->screenPos(), ev->delta(), ev->buttons(), ev->modifiers());
        combiner_manager.wheelEvent(&wheelEv);
        return true;
    } else if(event->type() == QEvent::KeyPress) {
        QKeyEvent* keyEv = dynamic_cast<QKeyEvent*>(event);
        combiner_manager.keyEvent(keyEv);
    }

    return Panel::eventFilter(target, event);
}

void ImageCombinerPanel::quit()
{
    worker->quit();
}

void ImageCombinerPanel::wait()
{
    worker->wait();
}


void ImageCombinerPanel::input_1(const cv::Mat img, const cv::Mat mask)
{
    has_1 = true;
    img1 = img;
    mask1 = mask;
    handle();
}

void ImageCombinerPanel::input_2(const cv::Mat img, const cv::Mat mask)
{
    has_2 = true;
    img2 = img;
    mask2 = mask;
    handle();
}

void ImageCombinerPanel::handle()
{
    if(has_1 && has_2) {
        has_1 = false;
        has_2 = false;

        Q_EMIT handle(img1, mask1, img2, mask2);
    }
}
