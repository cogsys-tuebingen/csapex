#ifndef REQUEST_SERIALIZER_H
#define REQUEST_SERIALIZER_H

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/io/io_fwd.h>

/// SYSTEM
#include <inttypes.h>

namespace csapex
{

class RequestSerializerInterface
{
public:
    virtual ~RequestSerializerInterface();

    virtual void serializeRequest(const RequestConstPtr& packet, SerializationBuffer &data) = 0;
    virtual RequestPtr deserializeRequest(SerializationBuffer& data) = 0;

    virtual void serializeResponse(const ResponseConstPtr& packet, SerializationBuffer &data) = 0;
    virtual ResponsePtr deserializeResponse(SerializationBuffer& data) = 0;
};


class RequestSerializer : public Singleton<RequestSerializer>, public Serializer
{
public:
    void serialize(const SerializableConstPtr& packet, SerializationBuffer &data) override;
    SerializablePtr deserialize(SerializationBuffer &data) override;

    static void registerSerializer(const std::string& type, std::shared_ptr<RequestSerializerInterface> serializer);

private:
    std::map<std::string, std::shared_ptr<RequestSerializerInterface>> serializers_;
};

template <typename S>
struct RequestSerializerRegistered
{
    RequestSerializerRegistered(const char* type) {
        RequestSerializer::registerSerializer(type, std::make_shared<S>());
    }
};
}


#endif // REQUEST_SERIALIZER_H
