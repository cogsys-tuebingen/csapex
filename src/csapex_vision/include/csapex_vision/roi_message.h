#ifndef ROI_MESSAGE_H
#define ROI_MESSAGE_H

/// PROJECT
#include <csapex/model/message.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
namespace connection_types {


struct RoiMessage : public MessageTemplate<cv::Rect, RoiMessage>
{
    RoiMessage();

    void writeYaml(YAML::Emitter& yaml);
    void readYaml(const YAML::Node& node);
};

}
}

#endif // ROI_MESSAGE_H
