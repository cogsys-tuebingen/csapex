/// HEADER
#include <csapex/serialization/parameter_serializer.h>

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/utility/assert.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/param/null_parameter.h>

/// SYSTEM
#include <iostream>

using namespace csapex;
using namespace csapex::param;

SerializerRegistered<ParameterSerializer> g_register_parameter_serializer_(Parameter::PACKET_TYPE_ID, &ParameterSerializer::instance());


ParameterSerializerInterface::~ParameterSerializerInterface()
{

}

void ParameterSerializer::serialize(const Streamable& packet, SerializationBuffer& data)
{
    if(const Parameter* parameter = dynamic_cast<const Parameter*>(&packet)) {
        uint8_t type = parameter->ID();
        auto it = serializers_.find(type);
        if(it != serializers_.end()) {
            data << type;

            // defer serialization to the corresponding serializer
            std::shared_ptr<ParameterSerializerInterface> serializer = it->second;
            serializer->serialize(*parameter, data);

        } else {
            std::cerr << "cannot serialize Parameter of type " << (int) type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
        }

    }
}

StreamablePtr ParameterSerializer::deserialize(const SerializationBuffer& data)
{
    uint8_t type;
    data >> type;

    if(type == param::NullParameter::NUMERICAL_ID) {
        return std::make_shared<param::NullParameter>();
    }

    auto it = serializers_.find(type);
    if(it != serializers_.end()) {
        // defer serialization to the corresponding serializer
        std::shared_ptr<ParameterSerializerInterface> serializer = it->second;
        return serializer->deserialize(data);

    } else {
        std::cerr << "cannot deserialize Parameter of type " << (int) type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
    }


    return ParameterPtr();
}

void ParameterSerializer::registerSerializer(uint8_t type, std::shared_ptr<ParameterSerializerInterface> serializer)
{
    instance().serializers_[type] = serializer;
}
