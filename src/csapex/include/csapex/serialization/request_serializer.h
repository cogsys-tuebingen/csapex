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
    virtual RequestPtr deserializeRequest(SerializationBuffer& data, uint8_t request_id) = 0;

    virtual void serializeResponse(const ResponseConstPtr& packet, SerializationBuffer &data) = 0;
    virtual ResponsePtr deserializeResponse(SerializationBuffer& data, uint8_t request_id) = 0;
};


class RequestSerializer : public Singleton<RequestSerializer>, public Serializer
{
public:
    void serialize(const StreamableConstPtr& packet, SerializationBuffer &data) override;
    StreamablePtr deserialize(SerializationBuffer &data) override;

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


#define CSAPEX_REGISTER_REQUEST_SERIALIZER(Name) \
    namespace csapex  \
    { \
    namespace io \
    { \
     \
    class Name##Serializer : public RequestSerializerInterface \
    { \
        virtual void serializeRequest(const RequestConstPtr& packet, SerializationBuffer &data) override \
        { \
            packet->serialize(data); \
        } \
        virtual RequestPtr deserializeRequest(SerializationBuffer& data, uint8_t request_id) override \
        { \
            auto result = std::make_shared<typename Name::RequestT>(request_id); \
            result->deserialize(data); \
            return result; \
        } \
        virtual void serializeResponse(const ResponseConstPtr& packet, SerializationBuffer &data) override \
        { \
            packet->serialize(data); \
        } \
        virtual ResponsePtr deserializeResponse(SerializationBuffer& data, uint8_t request_id) override \
        { \
            auto result = std::make_shared<typename Name::ResponseT>(request_id); \
            result->deserialize(data); \
            return result; \
        } \
    }; \
    } \
    RequestSerializerRegistered<io::Name##Serializer> g_register_##Name##_(#Name); \
    }


#endif // REQUEST_SERIALIZER_H
