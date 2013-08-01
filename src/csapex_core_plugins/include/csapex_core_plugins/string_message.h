#ifndef STRING_MESSAGE_H
#define STRING_MESSAGE_H

/// PROJECT
#include <csapex/message.h>

/// SYSTEM
#include <string>

namespace csapex {
namespace connection_types {

struct StringMessage : public MessageTemplate<std::string, StringMessage>
{
    StringMessage();

    static ConnectionType::Ptr make();

    void writeYaml(YAML::Emitter& yaml);
    void readYaml(YAML::Node& node);
};

}
}

#endif // STRING_MESSAGE_H
