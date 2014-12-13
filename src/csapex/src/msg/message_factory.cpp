/// HEADER
#include <csapex/msg/message_factory.h>

/// COMPONENT
#include <csapex/utility/assert.h>
#include <csapex/utility/yaml_node_builder.h>

/// SYSTEM
#include <fstream>

using namespace csapex;

MessageFactory::MessageFactory()
{
}

ConnectionType::Ptr MessageFactory::createMessage(const std::string& type)
{
    MessageFactory& i = instance();

    if(i.type_to_constructor.empty()) {
        throw std::runtime_error("no connection types registered!");
    }

    if(i.type_to_constructor.find(type) == i.type_to_constructor.end()) {
        throw std::runtime_error(std::string("no such type (") + type + ")");
    }

    return i.type_to_constructor[type]();
}

ConnectionType::Ptr MessageFactory::deserializeMessage(const YAML::Node &node)
{
    MessageFactory& i = instance();

    std::string type = node["type"].as<std::string>();

    if(i.type_to_constructor.empty()) {
        throw DeserializationError("no connection types registered!");
    }

    if(i.type_to_constructor.find(type) == i.type_to_constructor.end()) {
        throw DeserializationError(std::string("no such type (") + type + ")");
    }

    ConnectionType::Ptr msg = i.type_to_constructor[type]();
    try {
        i.type_to_converter.at(type).decoder(node["data"], *msg);
    } catch(const YAML::Exception& e) {
        throw DeserializationError(std::string("error while deserializing: ") + e.msg);
    }

    return msg;
}

YAML::Node MessageFactory::serializeMessage(const ConnectionType &msg)
{
    try {
        MessageFactory& i = instance();

        std::string type = msg.rawName();

        Converter& converter = i.type_to_converter.at(type);
        Converter::Encoder& encoder = converter.encoder;

        YAML::Node node;
        node["type"] = type;
        node["data"] = encoder(msg);

        return node;

    } catch(const std::out_of_range& e) {
        throw SerializationError(std::string("cannot serialize message of type ")
                                 + msg.name() + ", no YAML converter registered!");
    }
}

ConnectionType::Ptr MessageFactory::readMessage(const std::string &path)
{
    YAML::Node node = YAML::LoadFile(path);
    return readYaml(node);
}

void MessageFactory::writeMessage(const std::string &path, const ConnectionType& msg)
{
    std::ofstream out(path.c_str());

    YAML::Emitter yaml;
    yaml.SetSeqFormat(YAML::Flow);
    yaml << serializeMessage(msg);
    out << yaml.c_str();
}

ConnectionType::Ptr MessageFactory::readYaml(const YAML::Node &node)
{
    ConnectionType::Ptr msg = MessageFactory::deserializeMessage(node);
    if(!msg) {
        std::string type = node["type"].as<std::string>();
        throw DeserializationError(std::string("message type '") + type + "' unknown");
    }

    return msg;
}

void MessageFactory::registerMessage(std::string type, Constructor constructor, Converter converter)
{
    MessageFactory& i = instance();

    std::map<std::string, Constructor>::const_iterator it = i.type_to_constructor.find(type);

    apex_assert_hard(it == i.type_to_constructor.end());

    i.type_to_constructor.insert(std::make_pair(type, constructor));
    i.type_to_converter.insert(std::make_pair(type, converter));
}
