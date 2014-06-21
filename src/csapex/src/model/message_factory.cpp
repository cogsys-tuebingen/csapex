/// HEADER
#include <csapex/model/message_factory.h>

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

    if(i.classes.empty()) {
        throw std::runtime_error("no connection types registered!");
    }

    if(i.classes.find(type) == i.classes.end()) {
        throw std::runtime_error(std::string("no such type (") + type + ")");
    }

    return i.classes[type]();
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
    msg->write(out);
}

ConnectionType::Ptr MessageFactory::readYaml(const YAML::Node &node)
{
    std::string type;
    node["type"] >> type;

    ConnectionType::Ptr msg = MessageFactory::createMessage(type);
    if(!msg) {
        throw DeserializationError("message type unknown");
    }

    msg->readYaml(node);

    return msg;
}

void MessageFactory::registerMessage(Constructor constructor)
{
    MessageFactory& i = instance();

    std::string type = constructor()->name();
    std::map<std::string, Constructor>::const_iterator it = i.classes.find(type);
    apex_assert_hard(it == i.classes.end());
//    if(name != type) {
//        throw std::logic_error(name + " cannot be registered as a connection type, its name is different from the specified type: " + type);
//    }

    i.classes[type] = constructor;
}
