#ifndef COMMAND_SERIALIZER_H
#define COMMAND_SERIALIZER_H

/// PROJECT
#include <csapex/io/packet_serializer.h>
#include <csapex/command/command.h>

/// SYSTEM
#include <inttypes.h>

namespace csapex
{

class CommandSerializerInterface
{
public:
    virtual ~CommandSerializerInterface();

    virtual void serialize(const CommandPtr& packet, SerializationBuffer &data) = 0;
    virtual CommandPtr deserialize(SerializationBuffer& data) = 0;
};


class CommandSerializer : public Singleton<CommandSerializer>, public Serializer
{
public:
    void serialize(const SerializablePtr& packet, SerializationBuffer &data) override;
    SerializablePtr deserialize(SerializationBuffer &data) override;

    static void registerSerializer(const std::string& type, std::shared_ptr<CommandSerializerInterface> serializer);

private:
    std::map<std::string, std::shared_ptr<CommandSerializerInterface>> serializers_;
};

template <typename S>
struct CommandSerializerRegistered
{
    CommandSerializerRegistered(const char* type) {
        CommandSerializer::registerSerializer(type, std::make_shared<S>());
    }
};
}


#endif // COMMAND_SERIALIZER_H
