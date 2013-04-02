/// HEADER
#include "database_viewer.h"

/// COMPONENT
#include "database_item_model_decorator.h"
#include "ui_database_viewer.h"

/// PROJECT
#include <config/config.h>
#include <data/matchable_pose.h>
#include <db/database.h>
#include <db_strategy/factory.h>
#include <utils/LibUtil/QtCvImageConverter.h>

/// SYSTEM
#include <angles/angles.h>
#include <QTreeWidget>
#include <QTreeWidgetItem>

DatabaseViewer::DatabaseViewer(QWidget* parent) :
    DatabaseIOWindow(parent),
    ui(new Ui::DatabaseViewer)
{
    ui->setupUi(this);

    db = DatabaseStrategyFactory::create();

    if(db->loadConfig()) {
        db = DatabaseStrategyFactory::create();

        db_model = new DatabaseItemModelDecorator(db->getDatabase(), ui->tree);

        db->load();
        modelReplaced();
    }

    QObject::connect(ui->tree, SIGNAL(itemSelectionChanged()), this, SLOT(renderSelection()));
    QObject::connect(ui->angle_slider, SIGNAL(sliderMoved(int)), this, SLOT(renderAngle(int)));

    current_scene = new QGraphicsScene;
    ui->view->setScene(current_scene);
}

DatabaseViewer::~DatabaseViewer()
{
    delete ui;
    delete current_scene;
    delete db_model;
}

void DatabaseViewer::renderAngle(int angle)
{
    ui->tree->clearSelection();

    int index;
    renderPose(db->getPoseByAngle(angles::normalize_angle((angle - 180) / 180. * M_PI), &index));

//    ui->tree->setCurrentIndex(index);
}

void DatabaseViewer::renderPose(MatchablePose* p)
{
    QSharedPointer<QImage> img;

    int w = 320;
    int h = 240;

    if(p == MatchablePose::NULL_POSE) {
        img = QSharedPointer<QImage>(new QImage(w, h, QImage::Format_ARGB32));
        img->fill(0xFFFF0000);

    } else {
        cv::Mat tmp;
        cv::drawKeypoints(p->image, p->keypoints, tmp, cv::Scalar(50, 50, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        IplImage ipl_img = tmp;

        img = QtCvImageConverter::Converter<QImage, QSharedPointer>::ipl2QImage(&ipl_img);
    }

    current_scene->clear();
    current_scene->addPixmap(QPixmap::fromImage(img->scaled(w, h, Qt::KeepAspectRatioByExpanding)));
}

void DatabaseViewer::renderSelection()
{
    ERROR("reimplement");
    throw;

//    int i = ui->tree->currentIndex().row();

//    if(i >= 0 && i < db->getDatabase()->count()) {
//        renderPose(db->getDatabase()->getPose(i));
//    } else {
//        current_scene->clear();
//    }
}

void DatabaseViewer::deleteSelection()
{
    if(db_model->deleteCurrentRow()) {
        save_action->setEnabled(true);
    }
}
