#ifndef COMMAND_SERIALIZER_H
#define COMMAND_SERIALIZER_H

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/command/command.h>

/// SYSTEM
#include <inttypes.h>
#include <iostream>

namespace csapex
{
class CommandSerializerInterface
{
public:
    virtual ~CommandSerializerInterface();

    virtual void serialize(const Command& packet, SerializationBuffer& data) = 0;
    virtual CommandPtr deserialize(const SerializationBuffer& data) = 0;
};

class CommandSerializer : public Singleton<CommandSerializer>, public Serializer
{
public:
    void serialize(const Streamable& packet, SerializationBuffer& data) override;
    StreamablePtr deserialize(const SerializationBuffer& data) override;

    static void registerSerializer(const std::string& name, const std::string& type, std::shared_ptr<CommandSerializerInterface> serializer);

private:
    std::map<std::string, std::shared_ptr<CommandSerializerInterface>> serializers_;
};

template <typename S>
struct CommandSerializerRegistered
{
    CommandSerializerRegistered(const std::string& name, const std::string& type)
    {
        CommandSerializer::registerSerializer(name, type, std::make_shared<S>());
    }
};
}  // namespace csapex

#define CSAPEX_REGISTER_COMMAND_SERIALIZER(Name)                                                                                                                                                       \
    namespace csapex                                                                                                                                                                                   \
    {                                                                                                                                                                                                  \
    namespace command                                                                                                                                                                                  \
    {                                                                                                                                                                                                  \
    class Name##Serializer : public CommandSerializerInterface                                                                                                                                         \
    {                                                                                                                                                                                                  \
        virtual void serialize(const Command& packet, SerializationBuffer& data) override                                                                                                              \
        {                                                                                                                                                                                              \
            if (auto impl = dynamic_cast<const Name*>(&packet)) {                                                                                                                                      \
                impl->serializeVersioned(data);                                                                                                                                                        \
            }                                                                                                                                                                                          \
        }                                                                                                                                                                                              \
        virtual CommandPtr deserialize(const SerializationBuffer& data) override                                                                                                                       \
        {                                                                                                                                                                                              \
            CommandPtr res = std::make_shared<Name>();                                                                                                                                                 \
            res->deserializeVersioned(data);                                                                                                                                                           \
            return res;                                                                                                                                                                                \
        }                                                                                                                                                                                              \
    };                                                                                                                                                                                                 \
    }                                                                                                                                                                                                  \
    CommandSerializerRegistered<Name##Serializer> g_CSAPEX_REGISTER_COMMAND_SERIALIZER_##Name##_(#Name, Name::typeName());                                                                             \
    }

#endif  // COMMAND_SERIALIZER_H
