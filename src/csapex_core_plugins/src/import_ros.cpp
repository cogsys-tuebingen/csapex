/// HEADER
#include "import_ros.h"

/// COMPONENT
#include <csapex_core_plugins/ros_handler.h>
#include <csapex_core_plugins/ros_message_conversion.h>

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_out.h>
#include <csapex/manager/connection_type_manager.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/model/message.h>
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <yaml-cpp/eventhandler.h>
#include <sensor_msgs/Image.h>
#include <QAction>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>

CSAPEX_REGISTER_CLASS(csapex::ImportRos, csapex::BoxedObject)

using namespace csapex;

ImportRos::ImportRos()
    : connector_(NULL), topic_list(NULL)
{
    addTag(Tag::get("RosIO"));
    addTag(Tag::get("General"));
    addTag(Tag::get("Input"));
    setIcon(QIcon(":/terminal.png"));
}

void ImportRos::fill(QBoxLayout *layout)
{
    if(connector_ == NULL) {
        connector_ = box_->addOutput<connection_types::AnyMessage>("Something");

        dynamic_layout = new QVBoxLayout;
        layout->addLayout(dynamic_layout);

        QPushButton* refresh = new QPushButton("refresh");
        layout->addWidget(refresh);
        connect(refresh, SIGNAL(clicked()), this, SLOT(refresh()));
    }
}

void ImportRos::updateDynamicGui(QBoxLayout *layout)
{
    QtHelper::clearLayout(dynamic_layout);

    if(!state.topic_.empty()) {
        ROSHandler::instance().refresh();
    }

    if(ROSHandler::instance().nh()) {
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);

        int topic_count = 0;
        topic_list = new QComboBox;

        topic_list->addItem("");

        for(ros::master::V_TopicInfo::iterator it = topics.begin(); it != topics.end(); ++it) {
            topic_list->addItem(it->name.c_str());

            if(it->name == state.topic_) {
                topic_list->setCurrentIndex(topic_count + 1);
                changeTopic(QString(it->name.c_str()));
            }

            ++topic_count;
        }

        QObject::connect(topic_list, SIGNAL(currentIndexChanged(QString)), this, SLOT(changeTopic(QString)));

        if(topic_count == 0) {
            QLabel* label = new QLabel("no topics found");
            label->setStyleSheet("QLabel { font-style: italic; }");
            dynamic_layout->addWidget(label);

        } else {
            dynamic_layout->addWidget(topic_list);
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
    ROSHandler::instance().refresh();

    Q_EMIT modelChanged();
}

void ImportRos::messageArrived(ConnectorIn *source)
{
    // NO INPUT
}

void ImportRos::changeTopic(const QString& topic)
{
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for(ros::master::V_TopicInfo::iterator it = topics.begin(); it != topics.end(); ++it) {
        if(it->name == topic.toStdString()) {
            setTopic(*it);
            return;
        }
    }

    setError(true, std::string("cannot set topic, ") + topic.toStdString() + " doesn't exist.");
}


void ImportRos::setTopic(const ros::master::TopicInfo &topic)
{
    current_subscriber.shutdown();

    if(RosMessageConversion::instance().canHandle(topic)) {
        setError(false);
        state.topic_ = topic.name;

        std::cout << "warning: topic is " << topic.name << std::endl;
        current_subscriber = RosMessageConversion::instance().subscribe(topic, 1, connector_);

    } else {
        state.topic_ = "";
        setError(true, std::string("cannot import topic of type ") + topic.datatype);
        return;
    }

}


void ImportRos::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "topic" << YAML::Value << topic_;
}

void ImportRos::State::readYaml(const YAML::Node& node) {
    node["topic"] >> topic_;
}

Memento::Ptr ImportRos::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void ImportRos::setState(Memento::Ptr memento)
{
    boost::shared_ptr<ImportRos::State> m = boost::dynamic_pointer_cast<ImportRos::State> (memento);
    assert(m.get());

    state = *m;

    Q_EMIT modelChanged();
}

