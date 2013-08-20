/// HEADER
#include <csapex_vision/cv_mat_message.h>

using namespace csapex;
using namespace connection_types;


CvMatMessage::CvMatMessage()
    : MessageTemplate<cv::Mat, CvMatMessage> ("cv::Mat")
{}

ConnectionType::Ptr CvMatMessage::clone() {
    Ptr new_msg(new CvMatMessage);
    value.copyTo(new_msg->value);

    return new_msg;
}

ConnectionType::Ptr CvMatMessage::make(){
    Ptr new_msg(new CvMatMessage);
    return new_msg;
}
