/// HEADER
#include <csapex_vision/cv_mat_message.h>

using namespace csapex;
using namespace connection_types;


CvMatMessage::CvMatMessage()
    : MessageTemplate<cv::Mat, CvMatMessage> ("cv::Mat"), encoding(enc::bgr)
{}

ConnectionType::Ptr CvMatMessage::clone() {
    Ptr new_msg(new CvMatMessage);
    value.copyTo(new_msg->value);
    new_msg->encoding = encoding;
    return new_msg;
}

void CvMatMessage::writeRaw(const std::string &path, const std::string &suffix)
{
    std::string file = path + "/img" + suffix + ".jpg";
    cv::imwrite(file, value);
}
