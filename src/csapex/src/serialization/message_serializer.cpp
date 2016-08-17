/// HEADER
#include <csapex/serialization/message_serializer.h>

/// COMPONENT
#include <csapex/utility/assert.h>
#include <csapex/utility/yaml_node_builder.h>
#include <csapex/factory/message_factory.h>

/// SYSTEM
#include <fstream>
#include <iostream>

using namespace csapex;

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

TokenData::Ptr MessageSerializer::deserializeMessage(const YAML::Node &node)
{
    MessageSerializer& i = instance();

    std::string type;
    try {
        type = node["type"].as<std::string>();

    } catch(const std::exception& e) {
        std::cerr << "node: " << node << std::endl;
        throw DeserializationError("cannot get type");
    }

    if(i.type_to_converter.empty()) {
        throw DeserializationError("no connection types registered!");
    }

    if(i.type_to_converter.find(type) == i.type_to_converter.end()) {
        throw DeserializationError(std::string("no such type (") + type + ")");
    }

    TokenData::Ptr msg = MessageFactory::createMessage(type);
    try {
        i.type_to_converter.at(type).decoder(node["data"], *msg);
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
