/// HEADER
#include "import_ros.h"

/// COMPONENT
#include <csapex_core_plugins/ros_handler.h>
#include <csapex_core_plugins/ros_message_conversion.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/manager/connection_type_manager.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/model/message.h>
#include <csapex/utility/qt_helper.hpp>
#include <utils_param/parameter_factory.h>
#include <utils_param/set_parameter.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <yaml-cpp/eventhandler.h>
#include <sensor_msgs/Image.h>
#include <QAction>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>

CSAPEX_REGISTER_CLASS(csapex::ImportRos, csapex::Node)

using namespace csapex;

ImportRos::ImportRos()
    : connector_(NULL)
{
    addTag(Tag::get("RosIO"));
    addTag(Tag::get("General"));
    addTag(Tag::get("Input"));
    setIcon(QIcon(":/terminal.png"));

    std::vector<std::string> set;
    set.push_back("/");
    addParameter(param::ParameterFactory::declareParameterStringSet("topic", set),
                 boost::bind(&ImportRos::update, this));

    addParameter(param::ParameterFactory::declareTrigger("refresh"),
                 boost::bind(&ImportRos::refresh, this));
}

void ImportRos::setup()
{
    setSynchronizedInputs(true);
    connector_ = addOutput<connection_types::AnyMessage>("Something");

    refresh();
}

void ImportRos::refresh()
{
    ROSHandler::instance().refresh();

    if(ROSHandler::instance().nh()) {
        std::string old_topic = param<std::string>("topic");

        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);

        param::SetParameter::Ptr setp = boost::dynamic_pointer_cast<param::SetParameter>(getParameter("topic"));
        if(setp) {
            setError(false);
            std::vector<std::string> topics_str;
            for(ros::master::V_TopicInfo::const_iterator it = topics.begin(); it != topics.end(); ++it) {
                topics_str.push_back(it->name);
            }
            setp->setSet(topics_str);
            setp->set(old_topic);
            return;
        }
    }

    setError(true, "no ROS connection", EL_WARNING);
}

void ImportRos::update()
{
    ROSHandler::instance().refresh();

    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    std::string topic = param<std::string>("topic");

    for(ros::master::V_TopicInfo::iterator it = topics.begin(); it != topics.end(); ++it) {
        if(it->name == topic) {
            setTopic(*it);
            return;
        }
    }

    setError(true, std::string("cannot set topic, ") + topic + " doesn't exist.");
}

void ImportRos::process()
{
    // NO INPUT
}


void ImportRos::setTopic(const ros::master::TopicInfo &topic)
{
    if(topic.name == current_topic_) {
        return;
    }

    std::cout << "warning: set topic " << topic.name << std::endl;
    current_subscriber.shutdown();

    if(RosMessageConversion::instance().canHandle(topic)) {
        setError(false);

        std::cout << "warning: topic is " << topic.name << std::endl;
        current_topic_ = topic.name;
        current_subscriber = RosMessageConversion::instance().subscribe(topic, 1, connector_);

    } else {
        setError(true, std::string("cannot import topic of type ") + topic.datatype);
        return;
    }

}
