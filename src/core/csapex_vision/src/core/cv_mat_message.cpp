/// HEADER
#include <csapex_vision/cv_mat_message.h>

using namespace csapex;
using namespace connection_types;


CvMatMessage::CvMatMessage()
    : MessageTemplate<cv::Mat, CvMatMessage> ("cv::Mat"), encoding(enc::bgr)
{}

CvMatMessage::CvMatMessage(const Encoding& encoding)
    : MessageTemplate<cv::Mat, CvMatMessage> ("cv::Mat"), encoding(encoding)
{}

ConnectionType::Ptr CvMatMessage::clone() {
    Ptr new_msg(new CvMatMessage(encoding));
    value.copyTo(new_msg->value);
    return new_msg;
}

void CvMatMessage::writeRaw(const std::string &path, const std::string &suffix)
{
    std::string file = path + "/img" + suffix + ".jpg";
    cv::imwrite(file, value);
}

const Encoding& CvMatMessage::getEncoding() const
{
    return encoding;
}

void CvMatMessage::setEncoding(const Encoding &e)
{
    encoding = e;
}
