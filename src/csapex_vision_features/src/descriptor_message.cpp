/// HEADER
#include <csapex_vision_features/descriptor_message.h>

using namespace csapex;
using namespace connection_types;


DescriptorMessage::DescriptorMessage()
    : MessageTemplate<cv::Mat, DescriptorMessage> ("cv::Mat")
{}

