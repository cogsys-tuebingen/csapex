#ifndef POINT_CLOUD_MESSAGE_H
#define POINT_CLOUD_MESSAGE_H

/// PROJECT
#include <csapex/model/message.h>

/// SYSTEM
#include <tf/LinearMath/Transform.h>

namespace csapex {
namespace connection_types {


struct TransformMessage : public MessageTemplate<tf::Transform, TransformMessage>
{
    TransformMessage();

    void writeYaml(YAML::Emitter& yaml);
    void readYaml(YAML::Node& node);
};

}
}

#endif // POINT_CLOUD_MESSAGE_H
