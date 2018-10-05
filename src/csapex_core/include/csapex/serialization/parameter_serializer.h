#ifndef PARAMETER_SERIALIZER_H
#define PARAMETER_SERIALIZER_H

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/param/parameter.h>

/// SYSTEM
#include <inttypes.h>

namespace csapex
{
class ParameterSerializerInterface
{
public:
    virtual ~ParameterSerializerInterface();

    virtual void serialize(const param::Parameter& packet, SerializationBuffer& data) const = 0;
    virtual param::ParameterPtr deserialize(const SerializationBuffer& data) = 0;
};

class ParameterSerializer : public Singleton<ParameterSerializer>, public Serializer
{
    friend class Singleton<ParameterSerializer>;

public:
    ~ParameterSerializer() override;

    void serialize(const Streamable& packet, SerializationBuffer& data) override;
    StreamablePtr deserialize(const SerializationBuffer& data) override;

    static void registerSerializer(const std::string& type, std::shared_ptr<ParameterSerializerInterface> serializer);
    static void deregisterSerializer(const std::string& type);

protected:
    ParameterSerializer();

private:
    std::map<std::string, std::shared_ptr<ParameterSerializerInterface>> serializers_;
};

/// REGISTRATION

template <typename S, typename ParameterType>
struct ParameterSerializerRegistered
{
    ParameterSerializerRegistered()
    {
        ParameterSerializer::registerSerializer(param::serializationName<ParameterType>(), std::make_shared<S>());
    }
    ~ParameterSerializerRegistered()
    {
        ParameterSerializer::deregisterSerializer(param::serializationName<ParameterType>());
    }
};
}  // namespace csapex

#define CSAPEX_REGISTER_PARAMETER_SERIALIZER(Name)                                                                                                                                                     \
    namespace csapex                                                                                                                                                                                   \
    {                                                                                                                                                                                                  \
    namespace param                                                                                                                                                                                    \
    {                                                                                                                                                                                                  \
    class Name##Serializer : public ParameterSerializerInterface                                                                                                                                       \
    {                                                                                                                                                                                                  \
        virtual void serialize(const Parameter& packet, SerializationBuffer& data) const override                                                                                                      \
        {                                                                                                                                                                                              \
            packet.serializeVersioned(data);                                                                                                                                                           \
        }                                                                                                                                                                                              \
        virtual ParameterPtr deserialize(const SerializationBuffer& data) override                                                                                                                     \
        {                                                                                                                                                                                              \
            auto result = std::make_shared<Name>();                                                                                                                                                    \
            result->deserializeVersioned(data);                                                                                                                                                        \
            return result;                                                                                                                                                                             \
        }                                                                                                                                                                                              \
    };                                                                                                                                                                                                 \
    }                                                                                                                                                                                                  \
    ParameterSerializerRegistered<param::Name##Serializer, param::Name> g_register_##Name##_serializer;                                                                                                \
    }

#endif  // PARAMETER_SERIALIZER_H
