#ifndef ROS_MESSAGE_CONVERSION_H
#define ROS_MESSAGE_CONVERSION_H

/// COMPONENT
#include <csapex_core_plugins/ros_handler.h>
#include <csapex/connection_type.h>

/// SYSTEM
#include <ros/master.h>
#include <ros/subscriber.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

namespace csapex
{

class ConnectorOut;

class RosMessageConversion
{
public:
    class Convertor
    {
    public:
        typedef boost::shared_ptr<Convertor> Ptr;

    public:
        virtual ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, ConnectorOut* output) = 0;

    protected:
        void publish(ConnectorOut* output, csapex::ConnectionType::Ptr msg);
    };

    template <typename ROS, typename APEX, typename CONVERTION>
    class ConverterTemplate : public RosMessageConversion::Convertor
    {
        typedef ConverterTemplate<ROS, APEX, CONVERTION> Self;

        ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, ConnectorOut* output) {
            return ROSHandler::instance().nh()->subscribe<ROS>(topic.name, queue, boost::bind(&Self::callback, this, output, _1));
        }

        void callback(ConnectorOut* output, const typename ROS::ConstPtr& ros_msg) {
            typename APEX::Ptr apex_msg(new APEX);
            CONVERTION::convert(ros_msg, apex_msg);
            publish(output, apex_msg);
        }
    };

private:
    RosMessageConversion();

public:
    static RosMessageConversion& instance();

    static void registerConversion(const std::string& ros, Convertor::Ptr c);

    bool canHandle(const ros::master::TopicInfo &topic);

    ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, ConnectorOut *output);

private:
    void doRegisterConversion(const std::string& ros, Convertor::Ptr c);

private:
    std::map<std::string, Convertor::Ptr> converters_;
};

}

#endif // ROS_MESSAGE_CONVERSION_H
