#ifndef MESSAGES_DEFAULT_HPP
#define MESSAGES_DEFAULT_HPP

/// PROJECT
#include <designer/connection_type.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace vision_evaluator {
namespace connection_types {

struct Message : public ConnectionType
{
protected:
    Message(const std::string& name)
        : ConnectionType(name)
    {}

public:
    void writeYaml(YAML::Emitter& yaml) {

    }
    void readYaml(YAML::Node& node) {

    }
};

struct AnyMessage : public Message
{
protected:
    AnyMessage(const std::string& name)
        : Message(name)
    {}

public:
    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new AnyMessage("anything"));
        return new_msg;
    }
    virtual ConnectionType::Ptr toType() {
        Ptr new_msg(new AnyMessage("anything"));
        return new_msg;
    }

    static ConnectionType::Ptr make(){
        Ptr new_msg(new AnyMessage("anything"));
        return new_msg;
    }

    bool canConnectTo(Ptr other_side) {
        return true;
    }

    bool acceptsConnectionFrom(ConnectionType* other_side) {
        return true;
    }
};

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
