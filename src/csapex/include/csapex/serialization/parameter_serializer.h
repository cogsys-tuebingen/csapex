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

    virtual void serialize(const param::ParameterConstPtr& packet, SerializationBuffer &data) const = 0;
    virtual param::ParameterPtr deserialize(SerializationBuffer& data) = 0;
};


class ParameterSerializer : public Singleton<ParameterSerializer>, public Serializer
{
public:
    void serialize(const SerializableConstPtr &packet, SerializationBuffer &data) override;
    SerializablePtr deserialize(SerializationBuffer &data) override;

    static void registerSerializer(uint8_t type, std::shared_ptr<ParameterSerializerInterface> serializer);

private:
    std::map<int, std::shared_ptr<ParameterSerializerInterface>> serializers_;
};

template <typename S>
struct ParameterSerializerRegistered
{
    ParameterSerializerRegistered(int type) {
        ParameterSerializer::registerSerializer(type, std::make_shared<S>());
    }
};
}


#endif // PARAMETER_SERIALIZER_H
