/// HEADER
#include "image_provider_ros_topic.h"

/// SYSTEM
#include <QtConcurrentRun>
#include <QPushButton>
#include <QTimer>
#include <QLabel>
#include <ros/master.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//REGISTER_META_PLUGIN(ImageProviderRosTopic)

using namespace vision_evaluator;

boost::function<bool(ImageProvider*)> ImageProviderRosTopic::Identity
= boost::bind(&ImageProviderRosTopic::checkIdentity, _1);

QFuture<bool> ImageProviderRosTopic::has_connection;


void ImageProviderRosTopic::initHandle(bool try_only)
{
    if(try_only && has_connection.isRunning()) {
        std::cout << "init handle: still probing master" << std::endl;
        return;
    }

    if(has_connection.result()) {
        assert(nh == NULL);
        nh = new ros::NodeHandle("~");
    } else {
        std::cout << "init handle: no ros connection" << std::endl;
    }
}

ImageProviderRosTopic::ImageProviderRosTopic()
    : nh(NULL), layout(NULL)
{
    initHandle(true);
}

ImageProviderRosTopic::~ImageProviderRosTopic()
{
    if(nh != NULL) {
        nh->shutdown();
        delete nh;
    }
}

void ImageProviderRosTopic::checkMasterConnection()
{
    if(!ros::isInitialized()) {
        int c = 0;
        ros::init(c, (char**) NULL, "vision_evaluator");
    }

    if(!has_connection.isRunning()) {
        has_connection = QtConcurrent::run(ros::master::check);
    }
}

void ImageProviderRosTopic::init(int argc, char** argv)
{
    ros::init(argc, argv, "evaluator_node");

    checkMasterConnection();
}

void ImageProviderRosTopic::forceRefresh()
{
    has_connection.waitForFinished();

    std::cout << "refreshing topics" << std::endl;

    checkMasterConnection();

    if(nh != NULL) {
        // connection was there
        has_connection.waitForFinished();
        if(!has_connection.result()) {
            // connection no longer there
            nh->shutdown();
            delete nh;
            nh = NULL;
        }
    }

    initHandle();

    refresh();
}

void ImageProviderRosTopic::refresh()
{
    if(layout == NULL) {
        return;
    }

    // clear old stuff
    buttons.clear();

    QLayoutItem* item;

    while((item = layout->takeAt(0)) != NULL) {
        delete item->widget();
        delete item;
    }

    // if ros is connected: list all topics of type sensor_msgs/Image
    if(nh != NULL) {
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);

        for(ros::master::V_TopicInfo::iterator it = topics.begin(); it != topics.end(); ++it) {
            if(it->datatype == "sensor_msgs/Image") {
                std::cout << "topic: " << it->name << " / " << it->datatype << std::endl;
                QRadioButton* btn = new QRadioButton(it->name.c_str());
                layout->addWidget(btn);
                buttons.push_back(btn);
                QObject::connect(btn, SIGNAL(clicked()), this, SLOT(changeTopic()), Qt::QueuedConnection);
            }
        }

        if(buttons.size() == 0) {
            QLabel* label = new QLabel("no image topics found");
            label->setStyleSheet("QLabel { font-style: italic; }");
            layout->addWidget(label);
        }

    } else {
        QLabel* label = new QLabel("no connection to master");
        label->setStyleSheet("QLabel { color : red; }");
        layout->addWidget(label);
    }

    // add a refresh button
    QPushButton* refresh = new QPushButton(tr("refresh"));
    layout->addWidget(refresh);

    QObject::connect(refresh, SIGNAL(clicked()), this, SLOT(forceRefresh()));
}

void ImageProviderRosTopic::changeTopic()
{
    if(nh == NULL) {
        std::cout << "no ros connection" << std::endl;
        return;
    }

    for(unsigned i = 0; i < buttons.size(); ++i) {
        if(buttons[i]->isChecked()) {
            const char* topic = buttons[i]->text().toUtf8().constData();
            std::cout << "selected topic: "  << topic << std::endl;
            current_subscriber = nh->subscribe<sensor_msgs::Image>(topic, 1, boost::bind(&ImageProviderRosTopic::callback, this, _1));

//            queue_->viewer->setProvider(QSharedPointer<ImageProviderRosTopic>(this));
            return;
        }
    }

    current_subscriber.shutdown();
}

void ImageProviderRosTopic::callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr imgptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    const cv::Mat& img = imgptr->image;

    cv::Mat grown(img.rows + 80, img.cols + 80, img.type(), cv::Scalar::all(0));
    cv::Rect roi(40, 40, img.cols, img.rows);

    img.copyTo(cv::Mat(grown, roi));

    cv::Mat mask(grown.rows, grown.cols, CV_8UC1, cv::Scalar::all(0));

    int d = 5;
    cv::rectangle(mask, cv::Rect(roi.x+d, roi.y+d, roi.width-2*d, roi.height-2*d), cv::Scalar::all(255), CV_FILLED);

    new_image(grown, mask);
}

void ImageProviderRosTopic::init_gui(QToolBox* toolbox)
{
    QFrame* frame = new QFrame;
    layout = new QVBoxLayout;

    refresh();

    frame->setLayout(layout);
    toolbox->addItem(frame, tr("ROS Topic Input"));
}

bool ImageProviderRosTopic::checkIdentity(ImageProvider* other)
{
    return dynamic_cast<ImageProviderRosTopic*>(other) != NULL;
}

void ImageProviderRosTopic::next()
{
    ros::spinOnce();
}

bool ImageProviderRosTopic::hasNext()
{
    return true;
}
