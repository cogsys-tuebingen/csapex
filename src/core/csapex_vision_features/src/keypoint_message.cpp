/// HEADER
#include <csapex_vision_features/keypoint_message.h>

using namespace csapex;
using namespace connection_types;


KeypointMessage::KeypointMessage()
    : MessageTemplate<std::vector<cv::KeyPoint>, KeypointMessage> ("std::vector<cv::KeyPoint>")
{}
