/// HEADER
#include "import_ros.h"

/// PROJECT
#include <csapex/box.h>
#include <csapex/connector_out.h>
#include <csapex/connection_type_manager.h>
#include <csapex/stream_interceptor.h>
#include <csapex/message.h>
#include <csapex/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <yaml-cpp/eventhandler.h>
#include <sensor_msgs/Image.h>
#include <QAction>
#include <QComboBox>
#include <QPushButton>
#include <QtConcurrentRun>
#include <QLabel>

PLUGINLIB_EXPORT_CLASS(csapex::ImportRos, csapex::BoxedObject)

using namespace csapex;

ImportRos::ImportRos()
    : connector_(NULL), initialized_(false)
{
    setCategory("RosIO");
    setIcon(QIcon(":/terminal.png"));
}

void ImportRos::fill(QBoxLayout *layout)
{
    if(connector_ == NULL) {
        initHandle(true);

        connector_ = new ConnectorOut(box_, 0);
        connector_->setLabel("Something");
        connector_->setType(connection_types::AnyMessage::make());

        dynamic_layout = new QVBoxLayout;
        layout->addLayout(dynamic_layout);

        QPushButton* refresh = new QPushButton("refresh");
        layout->addWidget(refresh);
        connect(refresh, SIGNAL(clicked()), this, SLOT(refresh()));

        box_->addOutput(connector_);
    }
}

void ImportRos::updateDynamicGui(QBoxLayout *layout)
{
    QtHelper::clearLayout(dynamic_layout);

    if(nh) {
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);

        int topic_count = 0;
        QComboBox* combo = new QComboBox;

        combo->addItem("");

        for(ros::master::V_TopicInfo::iterator it = topics.begin(); it != topics.end(); ++it) {
            combo->addItem(it->name.c_str());

            if(it->name == topic_) {
                combo->setCurrentIndex(topic_count + 1);
            }

            ++topic_count;
        }

        QObject::connect(combo, SIGNAL(currentIndexChanged(QString)), this, SLOT(changeTopic(QString)));

        if(topic_count == 0) {
            QLabel* label = new QLabel("no topics found");
            label->setStyleSheet("QLabel { font-style: italic; }");
            dynamic_layout->addWidget(label);

        } else {
            dynamic_layout->addWidget(combo);
        }

    } else {
        QLabel* label = new QLabel("no connection to master");
        label->setStyleSheet("QLabel { color : red; }");
        dynamic_layout->addWidget(label);
    }
}

void ImportRos::tick()
{
}

void ImportRos::refresh()
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
        }
    }

    initHandle();

    Q_EMIT modelChanged();
}

void ImportRos::messageArrived(ConnectorIn *source)
{
    // NO INPUT
}

void ImportRos::checkMasterConnection()
{
    if(!ros::isInitialized()) {
        int c = 0;
        ros::init(c, (char**) NULL, "csapex");
    }

    if(!has_connection.isRunning()) {
        has_connection = QtConcurrent::run(ros::master::check);
    }
}

void ImportRos::changeTopic(const QString& topic)
{
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for(ros::master::V_TopicInfo::iterator it = topics.begin(); it != topics.end(); ++it) {
        if(it->name == topic.toUtf8().constData()) {
            setTopic(*it);
            return;
        }
    }
}

void ImportRos::setTopic(const ros::master::TopicInfo &topic)
{
    if(topic.datatype == "sensor_msgs/Image") {
        std::cout << "warning: topic is " << topic.name << std::endl;

        ros::SubscribeOptions so;

        current_subscriber = nh->subscribe(so);

//        nh->subscribe<sensor_msgs::Image>(topic_, 1, boost::bind(&ImportRos::callback<sensor_msgs::Image>, this));


    } else {
        setError(true, std::string("cannot import topic of type ") + topic.datatype);
        return;
    }

    setError(false);
    topic_ = topic.name;
}

template <class T>
void ImportRos::callback(const typename T::ConstPtr& msg)
{

}

void ImportRos::initHandle(bool try_only)
{
    if(!initialized_) {
        checkMasterConnection();
    }

    if(try_only && has_connection.isRunning()) {
        std::cout << "init handle: still probing master" << std::endl;
        return;
    }

    if(has_connection.result()) {
        nh.reset(new ros::NodeHandle("~"));
    } else {
        std::cout << "init handle: no ros connection" << std::endl;
    }
}
