#ifndef MESSAGES_DEFAULT_HPP
#define MESSAGES_DEFAULT_HPP

/// PROJECT
#include <designer/connection_type.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

struct CvMatMessage : public ConnectionType {
public:
    typedef boost::shared_ptr<CvMatMessage> Ptr;

    virtual ConnectionType::Ptr clone() {
        CvMatMessage::Ptr new_msg(new CvMatMessage);
        value.copyTo(new_msg->value);

        return new_msg;
    }

    cv::Mat value;
};

#endif // MESSAGES_DEFAULT_HPP
