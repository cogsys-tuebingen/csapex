/// HEADER
#include <csapex/serialization/parameter_serializer.h>

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/utility/assert.h>
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
#include <iostream>

using namespace csapex;
using namespace csapex::param;

SerializerRegistered<ParameterSerializer> g_register_parameter_serializer_(Parameter::PACKET_TYPE_ID, &ParameterSerializer::instance());


ParameterSerializerInterface::~ParameterSerializerInterface()
{

}

void ParameterSerializer::serialize(const SerializableConstPtr& packet, SerializationBuffer& data)
{
    if(const ParameterConstPtr& parameter = std::dynamic_pointer_cast<Parameter const>(packet)) {
//        std::cerr << "serializing Parameter" << std::endl;
        uint8_t type = parameter->ID();
        auto it = serializers_.find(type);
        if(it != serializers_.end()) {

//            std::cerr << "serializing Parameter (type=" << type << ")" << std::endl;
            data << type;

            // defer serialization to the corresponding serializer
            std::shared_ptr<ParameterSerializerInterface> serializer = it->second;
            serializer->serialize(parameter, data);

        } else {
            std::cerr << "cannot serialize Parameter of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
        }

    }
}

SerializablePtr ParameterSerializer::deserialize(SerializationBuffer& data)
{
//    std::cerr << "deserializing Parameter" << std::endl;

    uint8_t type;
    data >> type;

    auto it = serializers_.find(type);
    if(it != serializers_.end()) {

//        std::cerr << "deserializing Parameter (type=" << type << ")" << std::endl;

        // defer serialization to the corresponding serializer
        std::shared_ptr<ParameterSerializerInterface> serializer = it->second;
        return serializer->deserialize(data);

    } else {
        std::cerr << "cannot deserialize Parameter of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
    }


    return ParameterPtr();
}

void ParameterSerializer::registerSerializer(uint8_t type, std::shared_ptr<ParameterSerializerInterface> serializer)
{
//    std::cout << "registering serializer of type " << type << std::endl;
    instance().serializers_[type] = serializer;
}
