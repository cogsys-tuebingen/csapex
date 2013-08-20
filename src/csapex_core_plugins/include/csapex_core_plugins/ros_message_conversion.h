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
        virtual ros::Publisher advertise(const std::string& topic,  int queue, bool latch = false) = 0;

        virtual void publish(ros::Publisher& pub, ConnectionType::Ptr msg) = 0;

        virtual std::string rosType() = 0;
        virtual std::string apexType() = 0;

    protected:
        void publish_apex(ConnectorOut* output, csapex::ConnectionType::Ptr msg);
    };

    template <typename ROS, typename APEX, typename CONVERTION>
    class ConverterTemplate : public RosMessageConversion::Convertor
    {
        typedef ConverterTemplate<ROS, APEX, CONVERTION> Self;

    public:
        std::string rosType() {
            return ros::message_traits::DataType<ROS>::value();
        }
        std::string apexType() {
            return (APEX()).name();
        }

        ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, ConnectorOut* output) {
            return ROSHandler::instance().nh()->subscribe<ROS>(topic.name, queue, boost::bind(&Self::callback, this, output, _1));
        }
        ros::Publisher advertise(const std::string& topic, int queue, bool latch = false) {
            return ROSHandler::instance().nh()->advertise<ROS>(topic, queue, latch);
        }
        void publish(ros::Publisher& pub, ConnectionType::Ptr apex_msg_raw) {
            typename APEX::Ptr apex_msg = boost::dynamic_pointer_cast<APEX> (apex_msg_raw);
            typename ROS::Ptr ros_msg(new ROS);
            CONVERTION::apex2ros(apex_msg, ros_msg);
            return pub.publish(ros_msg);
        }

        void callback(ConnectorOut* output, const typename ROS::ConstPtr& ros_msg) {
            typename APEX::Ptr apex_msg(new APEX);
            CONVERTION::ros2apex(ros_msg, apex_msg);
            publish_apex(output, apex_msg);
        }
    };

private:
    RosMessageConversion();

public:
    static RosMessageConversion& instance();

    static void registerConversion(Convertor::Ptr c);

    bool canHandle(const ros::master::TopicInfo &topic);

    ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, ConnectorOut *output);
    ros::Publisher advertise(ConnectionType::Ptr, const std::string& topic,  int queue, bool latch = false);
    void publish(ros::Publisher& pub, ConnectionType::Ptr msg);


private:
    void doRegisterConversion(Convertor::Ptr c);

private:
    std::map<std::string, Convertor::Ptr> converters_;
    std::map<std::string, Convertor::Ptr> converters_inv_;
};

}

#endif // ROS_MESSAGE_CONVERSION_H
