/// HEADER
#include <csapex/command/command_serializer.h>

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/utility/assert.h>
#include <csapex/serialization/io/std_io.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

SerializerRegistered<CommandSerializer> g_CSAPEX_REGISTER_COMMAND_SERIALIZER_(Command::PACKET_TYPE_ID, &CommandSerializer::instance());

CommandSerializerInterface::~CommandSerializerInterface()
{
}

void CommandSerializer::serialize(const Streamable& packet, SerializationBuffer& data)
{
    if (const Command* cmd = dynamic_cast<const Command*>(&packet)) {
        std::string type = cmd->getType();
        data << type;

        // std::cerr << "trying to serialize command of type " << type << std::endl;

        auto it = serializers_.find(type);
        if (it != serializers_.end()) {
            // std::cerr << "serializing command (type=" << type << ")" << std::endl;

            // defer serialization to the corresponding serializer
            std::shared_ptr<CommandSerializerInterface> serializer = it->second;
            serializer->serialize(*cmd, data);

        } else {
            std::cerr << "cannot serialize command of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
        }
    }
}

StreamablePtr CommandSerializer::deserialize(const SerializationBuffer& data)
{
    std::string type;
    data >> type;

    // std::cerr << "trying to deserialize command of type " << type << std::endl;

    auto it = serializers_.find(type);
    if (it != serializers_.end()) {
        // std::cerr << "deserializing command (type=" << type << ")" << std::endl;

        // TODO: use boost serialization!
        // defer serialization to the corresponding serializer
        std::shared_ptr<CommandSerializerInterface> serializer = it->second;
        return serializer->deserialize(data);

    } else {
        std::cerr << "cannot deserialize command of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
    }

    return CommandPtr();
}

void CommandSerializer::registerSerializer(const std::string& name, const std::string& type, std::shared_ptr<CommandSerializerInterface> serializer)
{
    auto pos = instance().serializers_.find(type);
    if (pos != instance().serializers_.end()) {
        std::cerr << "Cannot register command '" << name << "': more than one command serializer of type " << type << " registered!" << std::endl;
        std::abort();
    }
    instance().serializers_[type] = serializer;
}
