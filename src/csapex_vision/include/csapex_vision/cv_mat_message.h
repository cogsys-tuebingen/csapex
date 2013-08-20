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
    CvMatMessage();
    virtual ConnectionType::Ptr clone();
    static ConnectionType::Ptr make();
};

}
}

#endif // MESSAGES_DEFAULT_HPP
