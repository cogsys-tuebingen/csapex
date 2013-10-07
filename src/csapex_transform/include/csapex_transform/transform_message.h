#ifndef TRANSFORM_MESSAGE_H
#define TRANSFORM_MESSAGE_H

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

#endif // TRANSFORM_MESSAGE_H
