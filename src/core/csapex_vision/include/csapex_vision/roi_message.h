#ifndef ROI_MESSAGE_H
#define ROI_MESSAGE_H

/// COMPONENT
#include <csapex_vision/roi.h>

/// PROJECT
#include <csapex/model/message.h>

namespace csapex {
namespace connection_types {


struct RoiMessage : public MessageTemplate<Roi, RoiMessage>
{
    RoiMessage();

    void writeYaml(YAML::Emitter& yaml);
    void readYaml(const YAML::Node& node);
};

}
}

#endif // ROI_MESSAGE_H
