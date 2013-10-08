/// HEADER
#include <csapex_transform/time_stamp_message.h>

/// SYSTEM
#include <boost/date_time.hpp>

using namespace csapex;
using namespace connection_types;

TimeStampMessage::TimeStampMessage()
    : MessageTemplate<ros::Time, TimeStampMessage> ("TimeStamp")
{}

void TimeStampMessage::writeYaml(YAML::Emitter& yaml) {
    yaml << YAML::Key << "time" << YAML::Value << boost::posix_time::to_simple_string(value.toBoost()) << YAML::EndSeq;;
}

void TimeStampMessage::readYaml(YAML::Node& node) {
    if(node.FindValue("time")) {
        std::string str;
        node["time"] >> str;
        value = ros::Time::fromBoost(boost::posix_time::time_from_string(str));
    }
}
