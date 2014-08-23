/// HEADER
#include <csapex/msg/message_factory.h>

/// COMPONENT
#include <csapex/utility/assert.h>

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

    std::string type;
    node["type"] >> type;

    if(i.type_to_constructor.empty()) {
        throw std::runtime_error("no connection types registered!");
    }

    if(i.type_to_constructor.find(type) == i.type_to_constructor.end()) {
        throw std::runtime_error(std::string("no such type (") + type + ")");
    }

    ConnectionType::Ptr msg = i.type_to_constructor[type]();
    i.type_to_converter.at(type).decoder(node["data"], *msg);

    return msg;
}

YAML::Node MessageFactory::serializeMessage(const ConnectionType::Ptr &msg)
{
    MessageFactory& i = instance();

    std::string type = msg->name();

    YAML::Node node;
    node["type"] = type;
    node["data"] = i.type_to_converter.at(type).encoder(*msg);

    return node;
}

ConnectionType::Ptr MessageFactory::readMessage(const std::string &path)
{
    std::ifstream f(path.c_str());

    YAML::Node doc;

    YAML::Parser parser(f);
    if(getNextDocument(parser, doc)) {
        return readYaml(doc);
    }

    throw DeserializationError("path '" + path + "' cannot be read.");
}

void MessageFactory::writeMessage(const std::string &path, const ConnectionType::Ptr msg)
{
    std::ofstream out(path.c_str());

    YAML::Emitter yaml;
    yaml << serializeMessage(msg);
    out << yaml.c_str();
}

ConnectionType::Ptr MessageFactory::readYaml(const YAML::Node &node)
{
    ConnectionType::Ptr msg = MessageFactory::deserializeMessage(node);
    if(!msg) {
        std::string type;
        node["type"] >> type;
        throw DeserializationError(std::string("message type '") + type + "' unknown");
    }

    return msg;
}

void MessageFactory::registerMessage(Constructor constructor, Converter converter)
{
    MessageFactory& i = instance();

    std::string type = constructor()->name();
    std::map<std::string, Constructor>::const_iterator it = i.type_to_constructor.find(type);
    apex_assert_hard(it == i.type_to_constructor.end());
//    if(name != type) {
//        throw std::logic_error(name + " cannot be registered as a connection type, its name is different from the specified type: " + type);
//    }

    i.type_to_constructor.insert(std::make_pair(type, constructor));
    i.type_to_converter.insert(std::make_pair(type, converter));
}
