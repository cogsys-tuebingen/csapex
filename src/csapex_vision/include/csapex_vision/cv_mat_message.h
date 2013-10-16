#ifndef MESSAGES_DEFAULT_HPP
#define MESSAGES_DEFAULT_HPP

/// COMPONENT
#include <csapex_vision/encoding.h>

/// PROJECT
#include <csapex/model/message.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
namespace connection_types {


struct CvMatMessage : public MessageTemplate<cv::Mat, CvMatMessage>
{
    CvMatMessage();
    virtual ConnectionType::Ptr clone();

    virtual void writeRaw(const std::string &file, const std::string &suffix);

    Encoding encoding;
};

}
}

#endif // MESSAGES_DEFAULT_HPP
