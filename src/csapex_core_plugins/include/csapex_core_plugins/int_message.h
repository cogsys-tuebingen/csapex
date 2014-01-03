#ifndef INT_MESSAGE_H
#define INT_MESSAGE_H

/// PROJECT
#include <csapex/model/message.h>

/// SYSTEM
#include <string>

namespace csapex {
namespace connection_types {

struct IntMessage : public MessageTemplate<int, IntMessage>
{
    IntMessage();

    static ConnectionType::Ptr make();

    void writeYaml(YAML::Emitter& yaml);
    void readYaml(const YAML::Node& node);
};

}
}

#endif // INT_MESSAGE_H
