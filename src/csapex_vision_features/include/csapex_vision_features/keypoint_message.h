#ifndef KEYPOINT_MESSAGE_H
#define KEYPOINT_MESSAGE_H

/// PROJECT
#include <csapex/message.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
namespace connection_types {


struct KeypointMessage : public MessageTemplate<std::vector<cv::KeyPoint>, KeypointMessage>
{
    KeypointMessage();
};

}
}

#endif // KEYPOINT_MESSAGE_H
