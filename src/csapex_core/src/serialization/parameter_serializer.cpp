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

ParameterSerializer::ParameterSerializer()
{
    // std::cout << "create ParameterSerializer" << std::endl;
}

ParameterSerializer::~ParameterSerializer()
{
    // std::cout << "destroy ParameterSerializer" << std::endl;
}

void ParameterSerializer::serialize(const Streamable& packet, SerializationBuffer& data)
{
    if (const Parameter* parameter = dynamic_cast<const Parameter*>(&packet)) {
        std::string type = parameter->getParameterType();
        auto it = serializers_.find(type);
        if (it != serializers_.end()) {
            data << type;

            // defer serialization to the corresponding serializer
            std::shared_ptr<ParameterSerializerInterface> serializer = it->second;
            serializer->serialize(*parameter, data);

        } else {
            data << serializationName<NullParameter>();
            std::cerr << "cannot serialize Parameter of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
        }
    }
}

StreamablePtr ParameterSerializer::deserialize(const SerializationBuffer& data)
{
    std::string type;
    data >> type;

    if (type == param::serializationName<NullParameter>()) {
        std::cerr << "Received null parameter" << std::endl;
        return std::make_shared<param::NullParameter>();
    }

    auto it = serializers_.find(type);
    if (it != serializers_.end()) {
        // defer serialization to the corresponding serializer
        std::shared_ptr<ParameterSerializerInterface> serializer = it->second;
        return serializer->deserialize(data);

    } else {
        std::cerr << "cannot deserialize Parameter of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
    }

    return ParameterPtr();
}

void ParameterSerializer::registerSerializer(const std::string& type, std::shared_ptr<ParameterSerializerInterface> serializer)
{
    if (auto* ps = dynamic_cast<ParameterSerializer*>(PacketSerializer::instance().getSerializer(Parameter::PACKET_TYPE_ID))) {
        // std::cout << "register " << type << std::endl;
        ps->serializers_[type] = serializer;
    } else {
        throw std::runtime_error(std::string("Cannot register ") + type + std::string(" serializer, the parameter serializer is not registered yet."));
    }
}
void ParameterSerializer::deregisterSerializer(const std::string& type)
{
    if (auto* ps = dynamic_cast<ParameterSerializer*>(PacketSerializer::instance().getSerializer(Parameter::PACKET_TYPE_ID))) {
        // std::cout << "deregister " << type << std::endl;
        ps->serializers_.erase(type);
    }
}
