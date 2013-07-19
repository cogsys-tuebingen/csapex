#ifndef MESSAGES_DEFAULT_HPP
#define MESSAGES_DEFAULT_HPP

/// PROJECT
#include <designer/connection_type.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace vision_evaluator {
namespace connection_types {

template <class Type, class Instance>
struct MessageTemplate : public ConnectionType {
    typedef boost::shared_ptr<Instance> Ptr;

    MessageTemplate(const std::string& name)
        : ConnectionType(name)
    {}

    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new Instance);
        new_msg->value = value;
        return new_msg;
    }

    Type value;
};

struct CvMatMessage : public MessageTemplate<cv::Mat, CvMatMessage>
{
    CvMatMessage()
        : MessageTemplate<cv::Mat, CvMatMessage> ("cv::Mat")
    {}

    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new CvMatMessage());
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
};

}
}

#endif // MESSAGES_DEFAULT_HPP
