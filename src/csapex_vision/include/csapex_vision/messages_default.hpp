#ifndef MESSAGES_DEFAULT_HPP
#define MESSAGES_DEFAULT_HPP

/// PROJECT
#include <csapex/message.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
namespace connection_types {

template <class Type, class Instance>
struct MessageTemplate : public Message {
    typedef boost::shared_ptr<Instance> Ptr;

    MessageTemplate(const std::string& name)
        : Message(name)
    {}

    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new Instance);
        new_msg->value = value;
        return new_msg;
    }

    virtual ConnectionType::Ptr toType() {
        Ptr new_msg(new Instance);
        return new_msg;
    }

    bool acceptsConnectionFrom(ConnectionType* other_side) {
        return name() == other_side->name();
    }

    void writeYaml(YAML::Emitter& yaml) {
        yaml << YAML::Key << "value" << YAML::Value << "not implemented";
    }
    void readYaml(YAML::Node& node) {
    }

    Type value;
};

struct CvMatMessage : public MessageTemplate<cv::Mat, CvMatMessage>
{
    CvMatMessage()
        : MessageTemplate<cv::Mat, CvMatMessage> ("cv::Mat")
    {}

    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new CvMatMessage);
        value.copyTo(new_msg->value);

        return new_msg;
    }

    static ConnectionType::Ptr make(){
        Ptr new_msg(new CvMatMessage);
        return new_msg;
    }
};

struct StringMessage : public MessageTemplate<std::string, StringMessage>
{
    StringMessage()
        : MessageTemplate<std::string, StringMessage> ("std::string")
    {}

    static ConnectionType::Ptr make(){
        Ptr new_msg(new StringMessage);
        return new_msg;
    }

    void writeYaml(YAML::Emitter& yaml) {
        yaml << YAML::Key << "value" << YAML::Value << value;
    }
    void readYaml(YAML::Node& node) {
        node["value"] >> value;
    }
};

}
}

#endif // MESSAGES_DEFAULT_HPP
