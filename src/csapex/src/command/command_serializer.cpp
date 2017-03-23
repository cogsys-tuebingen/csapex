/// HEADER
#include <csapex/command/command_serializer.h>

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/utility/assert.h>
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

SerializerRegistered<CommandSerializer> g_CSAPEX_REGISTER_COMMAND_SERIALIZER_(Command::PACKET_TYPE_ID, &CommandSerializer::instance());


CommandSerializerInterface::~CommandSerializerInterface()
{

}

void CommandSerializer::serialize(const SerializableConstPtr& packet, SerializationBuffer& data)
{
    if(const CommandConstPtr& cmd = std::dynamic_pointer_cast<Command const>(packet)) {
        std::cerr << "serializing command" << std::endl;
        std::string type = cmd->getType();
        auto it = serializers_.find(type);
        if(it != serializers_.end()) {

            std::cerr << "serializing command (type=" << type << ")" << std::endl;
            data << type;

            // defer serialization to the corresponding serializer
            std::shared_ptr<CommandSerializerInterface> serializer = it->second;
            serializer->serialize(cmd, data);

        } else {
            std::cerr << "cannot serialize command of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
        }

    }
}

SerializablePtr CommandSerializer::deserialize(SerializationBuffer& data)
{
    std::cerr << "deserializing command" << std::endl;

    std::string type;
    data >> type;

    auto it = serializers_.find(type);
    if(it != serializers_.end()) {

        std::cerr << "deserializing command (type=" << type << ")" << std::endl;

        // TODO: use boost serialization!
        // defer serialization to the corresponding serializer
        std::shared_ptr<CommandSerializerInterface> serializer = it->second;
        return serializer->deserialize(data);

    } else {
        std::cerr << "cannot sdeerialize command of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
    }


    return CommandPtr();
}

void CommandSerializer::registerSerializer(const std::string &type, std::shared_ptr<CommandSerializerInterface> serializer)
{
    std::cout << "registering serializer of type " << type << std::endl;
    instance().serializers_[type] = serializer;
}
