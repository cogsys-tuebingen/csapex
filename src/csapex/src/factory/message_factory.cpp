/// HEADER
#include <csapex/factory/message_factory.h>

/// COMPONENT
#include <csapex/utility/assert.h>
#include <csapex/utility/yaml_node_builder.h>
#include <csapex/serialization/message_serializer.h>

/// SYSTEM
#include <fstream>

using namespace csapex;

MessageFactory::MessageFactory()
{
}

Token::Ptr MessageFactory::createMessage(const std::string& type)
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

Token::Ptr MessageFactory::readMessage(const std::string &path)
{
    YAML::Node node = YAML::LoadFile(path);
    return MessageSerializer::instance().readYaml(node);
}

void MessageFactory::writeMessage(const std::string &path, const Token& msg)
{
    std::ofstream out(path.c_str());

    YAML::Emitter yaml;
    yaml << MessageSerializer::instance().serializeMessage(msg);
    out << yaml.c_str();
}

void MessageFactory::registerMessage(std::string type, Constructor constructor)
{
    MessageFactory& i = instance();

    std::map<std::string, Constructor>::const_iterator it = i.type_to_constructor.find(type);

    if(it != i.type_to_constructor.end()) {
        return;
    }

    apex_assert_hard(it == i.type_to_constructor.end());

    i.type_to_constructor.insert(std::make_pair(type, constructor));
}
