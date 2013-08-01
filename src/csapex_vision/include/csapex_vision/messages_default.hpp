#ifndef MESSAGES_DEFAULT_HPP
#define MESSAGES_DEFAULT_HPP

/// PROJECT
#include <csapex/message.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
namespace connection_types {


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

}
}

#endif // MESSAGES_DEFAULT_HPP
