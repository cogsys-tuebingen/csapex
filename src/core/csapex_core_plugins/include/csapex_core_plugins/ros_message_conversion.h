#ifndef ROS_MESSAGE_CONVERSION_H
#define ROS_MESSAGE_CONVERSION_H

/// COMPONENT
#include <csapex_core_plugins/ros_handler.h>
#include <csapex/model/message.h>

/// PROJECT
#include <utils_plugin/singleton.hpp>

/// SYSTEM
#include <ros/master.h>
#include <ros/subscriber.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

namespace csapex
{

class ConnectorOut;


template <typename T>
class RosMessageConversionT;

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


template <typename T>
class IdentityConvertor : public Convertor
{
    typedef IdentityConvertor<T> Self;
public:
    std::string rosType() {
        return ros::message_traits::DataType<T>::value();
    }
    std::string apexType() {
        return ros::message_traits::DataType<T>::value();
    }

    ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, ConnectorOut* output) {
        boost::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if(!nh) {
            throw std::runtime_error("no ros connection");
        }

        return nh->subscribe<T>(topic.name, queue, boost::bind(&Self::callback, this, output, _1));
    }
    ros::Publisher advertise(const std::string& topic, int queue, bool latch = false) {
        boost::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if(!nh) {
            throw std::runtime_error("no ros connection");
        }

        return nh->advertise<T>(topic, queue, latch);
    }
    void publish(ros::Publisher& pub, ConnectionType::Ptr apex_msg_raw) {
        typename connection_types::GenericMessage<T>::Ptr msg =
                boost::dynamic_pointer_cast<connection_types::GenericMessage<T> > (apex_msg_raw);
        if(!msg) {
            throw std::runtime_error("trying to publish an empty message");
        }
        return pub.publish(msg->value);
    }

    void callback(ConnectorOut* output, const typename T::ConstPtr& ros_msg) {
        if(!ros_msg) {
            throw std::runtime_error("received an empty ros message");
        }
        typename connection_types::GenericMessage<T>::Ptr apex_msg(new connection_types::GenericMessage<T>);
        apex_msg->value.reset(new T(*ros_msg));
        publish_apex(output, apex_msg);
    }
};

template <typename ROS, typename APEX, typename Converter>
class ConverterTemplate : public Convertor
{
    typedef ConverterTemplate<ROS, APEX, Converter> Self;

public:
    std::string rosType() {
        return ros::message_traits::DataType<ROS>::value();
    }
    std::string apexType() {
        return (APEX()).name();
    }

    ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, ConnectorOut* output) {
        boost::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if(!nh) {
            throw std::runtime_error("no ros connection");
        }

        return nh->subscribe<ROS>(topic.name, queue, boost::bind(&Self::callback, this, output, _1));
    }
    ros::Publisher advertise(const std::string& topic, int queue, bool latch = false) {
        boost::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if(!nh) {
            throw std::runtime_error("no ros connection");
        }

        return nh->advertise<ROS>(topic, queue, latch);
    }
    void publish(ros::Publisher& pub, ConnectionType::Ptr apex_msg_raw) {
        typename APEX::Ptr apex_msg = boost::dynamic_pointer_cast<APEX> (apex_msg_raw);
        if(!apex_msg->isValid()) {
            throw std::runtime_error("trying to publish an empty message");
        }
        typename ROS::Ptr ros_msg(new ROS);
        Converter::apex2ros(apex_msg, ros_msg);
        return pub.publish(ros_msg);
    }

    void callback(ConnectorOut* output, const typename ROS::ConstPtr& ros_msg) {
        if(!ros_msg) {
            throw std::runtime_error("received an empty ros message");
        }
        typename APEX::Ptr apex_msg(new APEX);
        Converter::ros2apex(ros_msg, apex_msg);
        publish_apex(output, apex_msg);
    }
};

class RosMessageConversion : public Singleton<RosMessageConversion>
{
    friend class Singleton<RosMessageConversion>;

    template <typename T>
    friend class RosMessageConversionT;

private:
    RosMessageConversion();

public:
    template <typename ROS, typename APEX, typename Converter>
    static void registerConversion() {
        instance().doRegisterConversion(Convertor::Ptr(new ConverterTemplate<ROS, APEX, Converter>));
    }

    bool canHandle(const ros::master::TopicInfo &topic);

    ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, ConnectorOut *output);
    ros::Publisher advertise(ConnectionType::Ptr, const std::string& topic,  int queue, bool latch = false);
    void publish(ros::Publisher& pub, ConnectionType::Ptr msg);

private:
    void doRegisterConversion(Convertor::Ptr c);

    template <typename T>
    bool doCanConvert(typename boost::enable_if<ros::message_traits::IsMessage<T> >::type* dummy = 0) {
        return converters_inv_.find(ros::message_traits::DataType<T>::value()) != converters_inv_.end();
    }
    template <typename T>
    bool doCanConvert(typename boost::disable_if<ros::message_traits::IsMessage<T> >::type* dummy = 0) {
        return false;
    }

private:
    std::map<std::string, Convertor::Ptr> converters_;
    std::map<std::string, Convertor::Ptr> converters_inv_;
};


template <typename T>
class RosMessageConversionT
{
public:
    template <typename U>
    static void registerConversionImpl(typename boost::enable_if<ros::message_traits::IsMessage<U> >::type* dummy = 0) {
        if(!canConvert()) {
            RosMessageConversion::instance().doRegisterConversion(Convertor::Ptr(new IdentityConvertor<T>));
        }
    }
    template <typename U>
    static void registerConversionImpl(typename boost::disable_if<ros::message_traits::IsMessage<U> >::type* dummy = 0) {
    }

    static void registerConversion() {
        registerConversionImpl<T>();
    }

    static bool canConvert() {
        return RosMessageConversion::instance().doCanConvert<T>();
    }
};


}

#endif // ROS_MESSAGE_CONVERSION_H
