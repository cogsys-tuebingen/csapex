/// HEADER
#include <csapex/serialization/message_serializer.h>

/// COMPONENT
#include <csapex/utility/assert.h>
#include <csapex/utility/yaml_node_builder.h>
#include <csapex/factory/message_factory.h>
#include <csapex/serialization/io/std_io.h>

/// SYSTEM
#include <fstream>
#include <iostream>

using namespace csapex;

SerializerRegistered<MessageSerializer> g_register_msg_serializer_(TokenData::PACKET_TYPE_ID, &MessageSerializer::instance());

MessageSerializer::MessageSerializer()
{
}

MessageSerializer::~MessageSerializer()
{

}

void MessageSerializer::shutdown()
{
	type_to_converter.clear();
}
void MessageSerializer::serialize(const Streamable& packet, SerializationBuffer& data)
{
    if(const TokenData* message = dynamic_cast<const TokenData*>(&packet)) {
        std::string type = message->typeName();
        data << type;

        message->serialize(data);
    }
}

StreamablePtr MessageSerializer::deserialize(const SerializationBuffer& data)
{
    std::string type;
    data >> type;

    TokenData::Ptr result = MessageFactory::createMessage(type);
    apex_assert_hard(result);
    result->deserialize(data);

    return result;
}

TokenData::Ptr MessageSerializer::deserializeMessage(const YAML::Node &node)
{
    MessageSerializer& i = instance();

    std::string type;
    try {
        type = node["type"].as<std::string>();

    } catch(const std::exception& e) {
        throw DeserializationError("cannot get type");
    }

    if(i.type_to_converter.empty()) {
        throw DeserializationError("no connection types registered!");
    }

    std::string converter_type = type;
    const std::string ns = "csapex::connection_types::";
    if(type.find(ns) != std::string::npos){
        converter_type.erase(0,ns.size());
    }

    if(i.type_to_converter.find(converter_type) == i.type_to_converter.end()) {
        throw DeserializationError(std::string("cannot deserialize, no such type (") + type + ")");
    }

    TokenData::Ptr msg = MessageFactory::createMessage(converter_type);
    try {
        i.type_to_converter.at(converter_type).decoder(node["data"], *msg);
    } catch(const YAML::Exception& e) {
        throw DeserializationError(std::string("error while deserializing: ") + e.msg);
    }

    return msg;
}

YAML::Node MessageSerializer::serializeMessage(const TokenData &msg)
{
    try {
        MessageSerializer& i = instance();

        std::string type = msg.typeName();

        YAML::Node node;
        auto pos = i.type_to_converter.find(type);
        if(pos == i.type_to_converter.end()) {
            return node;
        }

        Converter& converter = i.type_to_converter.at(type);
        Converter::Encoder& encoder = converter.encoder;

        node["type"] = type;
        node["data"] = encoder(msg);

        return node;

    } catch(const std::out_of_range& e) {
        throw SerializationError(std::string("cannot serialize message of type ")
                                 + msg.descriptiveName() + ", no YAML converter registered for " + msg.typeName());
    }
}


TokenData::Ptr MessageSerializer::readYaml(const YAML::Node &node)
{
    TokenData::Ptr msg = MessageSerializer::deserializeMessage(node);
    if(!msg) {
        std::string type = node["type"].as<std::string>();
        throw DeserializationError(std::string("message type '") + type + "' unknown");
    }

    return msg;
}

void MessageSerializer::registerMessage(std::string type, Converter converter)
{
    MessageSerializer& i = instance();

    std::map<std::string, Converter>::const_iterator it = i.type_to_converter.find(type);

    if(it != i.type_to_converter.end()) {
        return;
    }

    apex_assert_hard(it == i.type_to_converter.end());

    i.type_to_converter.insert(std::make_pair(type, converter));
}
