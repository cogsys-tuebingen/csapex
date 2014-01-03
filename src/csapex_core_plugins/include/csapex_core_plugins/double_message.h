#ifndef DOUBLE_MESSAGE_H
#define DOUBLE_MESSAGE_H

/// PROJECT
#include <csapex/model/message.h>

/// SYSTEM
#include <string>

namespace csapex {
namespace connection_types {

struct DoubleMessage : public MessageTemplate<double, DoubleMessage>
{
    DoubleMessage();

    static ConnectionType::Ptr make();

    void writeYaml(YAML::Emitter& yaml);
    void readYaml(const YAML::Node& node);
};

}
}

#endif // DOUBLE_MESSAGE_H
