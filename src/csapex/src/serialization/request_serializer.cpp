/// HEADER
#include <csapex/serialization/request_serializer.h>

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/utility/assert.h>
#include <csapex/io/request.h>
#include <csapex/io/response.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

SerializerRegistered<RequestSerializer> g_register_request_serializer_(Request::PACKET_TYPE_ID, &RequestSerializer::instance());
SerializerRegistered<RequestSerializer> g_register_response_serializer_(Response::PACKET_TYPE_ID, &RequestSerializer::instance());


RequestSerializerInterface::~RequestSerializerInterface()
{

}

void RequestSerializer::serialize(const StreamableConstPtr &packet, SerializationBuffer& data)
{
    if(const RequestConstPtr& request = std::dynamic_pointer_cast<Request const>(packet)) {
        std::string type = request->getType();
        auto it = serializers_.find(type);
        if(it != serializers_.end()) {

           // std::cerr << "serializing Request (type=" << type << ")" << std::endl;
            data << type;

            data << (uint8_t) 0;

            data << request->getRequestID();

            // defer serialization to the corresponding serializer
            std::shared_ptr<RequestSerializerInterface> serializer = it->second;
            serializer->serializeRequest(request, data);

        } else {
            std::cerr << "cannot serialize Request of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
        }

    } else if(const ResponseConstPtr& response = std::dynamic_pointer_cast<Response const>(packet)) {
        std::string type = response->getType();
        auto it = serializers_.find(type);
        if(it != serializers_.end()) {

            //std::cerr << "serializing Response (type=" << type << ")" << std::endl;
            data << type;

            data << (uint8_t) 1;

            data << response->getRequestID();

            // defer serialization to the corresponding serializer
            std::shared_ptr<RequestSerializerInterface> serializer = it->second;
            serializer->serializeResponse(response, data);

        } else {
            std::cerr << "cannot serialize Response of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
        }
    }
}

StreamablePtr RequestSerializer::deserialize(const SerializationBuffer& data)
{
//    std::cerr << "deserializing Request" << std::endl;

    std::string type;
    data >> type;

    auto it = serializers_.find(type);
    if(it != serializers_.end()) {

        uint8_t direction;
        data >> direction;

        uint8_t id;
        data >> id;

        if(direction == 0) {
            //std::cerr << "deserializing Request (type=" << type << ")" << std::endl;

            // defer serialization to the corresponding serializer
            std::shared_ptr<RequestSerializerInterface> serializer = it->second;
            return serializer->deserializeRequest(data, id);

        } else if(direction == 1) {
            //std::cerr << "deserializing Response (type=" << type << ")" << std::endl;

            // defer serialization to the corresponding serializer
            std::shared_ptr<RequestSerializerInterface> serializer = it->second;
            return serializer->deserializeResponse(data, id);

        }
    }
    std::cerr << "cannot deserialize Request of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;


    return RequestPtr();
}

void RequestSerializer::registerSerializer(const std::string &type, std::shared_ptr<RequestSerializerInterface> serializer)
{
//    std::cout << "registering serializer of type " << type << std::endl;
    instance().serializers_[type] = serializer;
}
