#ifndef MESSAGES_DEFAULT_HPP
#define MESSAGES_DEFAULT_HPP

/// PROJECT
#include <designer/connection_type.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace vision_evaluator {
namespace connection_types {

template <class Type>
struct MessageTemplate : public ConnectionType {
    typedef MessageTemplate<Type> Instance;
    typedef boost::shared_ptr<Instance> Ptr;

    MessageTemplate(const std::string& name)
        : ConnectionType(name)
    {}

    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new Instance(name()));
        *new_msg = *this;
        return new_msg;
    }

    Type value;
};

struct CvMatMessage : public MessageTemplate<cv::Mat>
{
    CvMatMessage()
        : MessageTemplate<cv::Mat> ("cv::Mat")
    {}

    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new Instance(name()));
        value.copyTo(new_msg->value);

        return new_msg;
    }

};

struct StringMessage : public MessageTemplate<std::string>
{
    StringMessage()
        : MessageTemplate<std::string> ("std::string")
    {}
};

}
}

#endif // MESSAGES_DEFAULT_HPP
