/// HEADER
#include <csapex_core_plugins/ros_message_conversion.h>

/// PROJECT
#include <csapex/connector_out.h>

using namespace csapex;

RosMessageConversion::RosMessageConversion()
{
}

RosMessageConversion& RosMessageConversion::instance()
{
    static RosMessageConversion instance;
    return instance;
}

bool RosMessageConversion::canHandle(const ros::master::TopicInfo &topic)
{
    return converters_.find(topic.datatype) != converters_.end();
}

void RosMessageConversion::registerConversion(const std::string &ros, Convertor::Ptr c)
{
    instance().doRegisterConversion(ros, c);
}

void RosMessageConversion::doRegisterConversion(const std::string &ros, Convertor::Ptr c)
{
    converters_.insert(std::make_pair(ros, c));
}

ros::Subscriber RosMessageConversion::subscribe(const ros::master::TopicInfo &topic, int queue, ConnectorOut* output)
{
    return converters_.at(topic.datatype)->subscribe(topic, queue, output);
}

void RosMessageConversion::Convertor::publish(ConnectorOut *output, ConnectionType::Ptr msg)
{
    output->setType(msg);
    output->publish(msg);
}
