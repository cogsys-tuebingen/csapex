/// HEADER
#include "analyzer_window.h"

/// COMPONENT
#include "database_item_model_decorator.h"
#include "ui_analyzer_window.h"

/// PROJECT
#include <adapter/analyzer_adapter.h>
#include <adapter/trainer_adapter.h>
#include <analyzer/trainer.h>
#include <data/frame_buffer.h>
#include <db_strategy/db_strategy.h>
#include <utils/LibUtil/QtCvImageConverter.h>

/// SYSTEM
#include <QPushButton>
#include <QGraphicsView>
#include <QStatusBar>
#include <QTime>
#include <QLayout>
#include <QSharedPointer>

void AnalyzerThread::run()
{
    analyzer_adapter.runHeadless();
}

AnalyzerWindow::AnalyzerWindow(AnalyzerAdapter* analyzer_node, QWidget* parent)
    : DatabaseIOWindow(parent),
      ui(new Ui::AnalyzerWindow),
      analyzer(analyzer_node->getAnalyzer()),
      analyzer_adapter(analyzer_node),
      show_controls(false)
{
    ui->setupUi(this);

    /// TODO: ugly
    Trainer* trainer = dynamic_cast<Trainer*>(&analyzer);
    if(trainer) {
        show_controls = true;
        ui->train_btn->setChecked(trainer->getState() == Trainer::TRAINING);
        ui->detect_btn->setEnabled(trainer->getState() != Trainer::TRAINING);
    }

    thread = new AnalyzerThread(*analyzer_node);
    thread->start();

    setWindowTitle(analyzer.getName().c_str());

    db_model = new DatabaseItemModelDecorator(analyzer.getDatabase(), ui->tree);
    modelReplaced();

    analyzer_connection = analyzer.frame_analyzed.connect(boost::bind(&AnalyzerWindow::triggerUpdateViewRequest, this, _1));

    qRegisterMetaType<Frame::Ptr >("Frame::Ptr");

    QObject::connect(this, SIGNAL(updateViewRequest(Frame::Ptr)),
                     this, SLOT(updateView(Frame::Ptr)), Qt::QueuedConnection);

    QObject::connect(ui->train_btn, SIGNAL(clicked()),
                     this, SLOT(changeState()));

    QObject::connect(ui->detect_btn, SIGNAL(clicked()),
                     this, SLOT(changeState()));

    db_model->rebuild();

    if(!show_controls) {
        QLayoutItem* item;
        while((item = ui->menu->takeAt(0)) != NULL) {
            delete item->widget();
            delete item;
        }
        delete ui->menu;
    }
}

void AnalyzerWindow::triggerUpdateViewRequest(Frame::Ptr current_frame)
{
    Q_EMIT updateViewRequest(current_frame);
}

AnalyzerWindow::~AnalyzerWindow()
{
    analyzer_connection.disconnect();

    delete ui;
    for(unsigned i=0; i < scenes.size(); ++i) {
        delete scenes[i];
    }
    for(unsigned i=0; i < tabs.size(); ++i) {
        delete tabs[i];
    }

    delete db_model;
}

void AnalyzerWindow::shutdown()
{
    analyzer_adapter->shutdown();
}

void AnalyzerWindow::changeState()
{
    if(!show_controls) {
        return;
    }

    Trainer* trainer = reinterpret_cast<Trainer*>(&analyzer);

    if(ui->detect_btn->isChecked()) {
        trainer->stopTraining();
        ui->train_btn->setEnabled(false);

    } else {
        ui->train_btn->setEnabled(true);

        if(ui->train_btn->isChecked()) {
            trainer->startTraining();
        } else {
            trainer->pauseTraining();
        }
    }
}

void AnalyzerWindow::updateView(Frame::Ptr current_frame)
{
    if(!current_frame->isValid()) {
        INFO("got an update view request with an invalid frame.");
        return;
    }

    cv::Mat img = current_frame->getDebugImage();

    setView(0, img, "debug");

    int extras = FrameBuffer::getImageCount();
    for(int i = 0; i < extras; ++i) {
        img = FrameBuffer::getImage(i);
        setView(i + 1, img, FrameBuffer::getName(i));
    }

    resize(sizeHint());
}

void AnalyzerWindow::setView(int index, cv::Mat& i, const std::string& name)
{
    if(i.rows == 0 || i.cols == 0) {
        return;
    }

    while(index >= (int) scenes.size()) {
        QGraphicsScene* s = new QGraphicsScene;
        QGraphicsView* v = new QGraphicsView(s);
        scenes.push_back(s);
        tabs.push_back(v);

        ui->tabs->addTab(v, "");
    }

    ui->tabs->setTabText(index, name.c_str());

    QSharedPointer<QImage> img = QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(i);
    QPixmap pm = QPixmap::fromImage(*img);

    scenes[index]->clear();
    scenes[index]->addPixmap(pm);
}
