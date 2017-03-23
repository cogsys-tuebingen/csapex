#ifndef COMMAND_SERIALIZER_H
#define COMMAND_SERIALIZER_H

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/command/command.h>

/// SYSTEM
#include <inttypes.h>

namespace csapex
{

class CommandSerializerInterface
{
public:
    virtual ~CommandSerializerInterface();

    virtual void serialize(const CommandConstPtr& packet, SerializationBuffer &data) = 0;
    virtual CommandPtr deserialize(SerializationBuffer& data) = 0;
};


class CommandSerializer : public Singleton<CommandSerializer>, public Serializer
{
public:
    void serialize(const SerializableConstPtr& packet, SerializationBuffer &data) override;
    SerializablePtr deserialize(SerializationBuffer &data) override;

    static void registerSerializer(const std::string& type, std::shared_ptr<CommandSerializerInterface> serializer);

private:
    std::map<std::string, std::shared_ptr<CommandSerializerInterface>> serializers_;
};

template <typename S>
struct CommandSerializerRegistered
{
    CommandSerializerRegistered(const std::string& type) {
        CommandSerializer::registerSerializer(type, std::make_shared<S>());
    }
};
}


#define CSAPEX_REGISTER_COMMAND_SERIALIZER(Name) \
    namespace csapex\
    {\
    namespace command\
    {\
    \
    class Name##Serializer : public CommandSerializerInterface\
    {\
        virtual void serialize(const CommandConstPtr& packet, SerializationBuffer &data) override\
        {\
            if(auto impl = std::dynamic_pointer_cast<Name const>(packet)) {\
                impl->serialize(data);\
            }\
        }\
        virtual CommandPtr deserialize(SerializationBuffer& data) override\
        {\
            CommandPtr res = std::make_shared<Name>();\
            res->deserialize(data);\
            return res;\
        }\
    };\
    }\
    CommandSerializerRegistered<Name##Serializer> g_CSAPEX_REGISTER_COMMAND_SERIALIZER_##Name##_(Name::typeName());\
    }

#endif // COMMAND_SERIALIZER_H
