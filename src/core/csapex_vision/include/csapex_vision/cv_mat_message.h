#ifndef CV_MAT_MESSAGE_H
#define CV_MAT_MESSAGE_H

/// COMPONENT
#include <csapex_vision/encoding.h>

/// PROJECT
#include <csapex/model/message.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {

template <typename,typename,typename> class ConverterTemplate;

namespace connection_types {

struct CvMatMessage : public MessageTemplate<cv::Mat, CvMatMessage>
{
    friend class MessageTemplate<cv::Mat, CvMatMessage>;

    template <typename,typename,typename> friend class csapex::ConverterTemplate;

public:
    CvMatMessage(const Encoding& encoding);
    virtual ConnectionType::Ptr clone();

    virtual void writeRaw(const std::string &file, const std::string &suffix);

    const Encoding &getEncoding() const;
    void setEncoding(const Encoding& e);

private:
    Encoding encoding;

private:
    CvMatMessage();
};

}
}

#endif // CV_MAT_MESSAGE_H
