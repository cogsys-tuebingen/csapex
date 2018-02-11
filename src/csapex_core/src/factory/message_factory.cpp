/// HEADER
#include <csapex/factory/message_factory.h>

/// COMPONENT
#include <csapex/utility/assert.h>
#include <csapex/utility/yaml_node_builder.h>
#include <csapex/serialization/message_serializer.h>
#include <csapex/core/settings.h>

/// SYSTEM
#include <fstream>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <iostream>

using namespace csapex;

MessageFactory::MessageFactory()
{
}

void MessageFactory::shutdown()
{
    type_to_constructor.clear();
}

TokenData::Ptr MessageFactory::createMessage(const std::string& type)
{
    MessageFactory& i = instance();

    if(i.type_to_constructor.empty()) {
        throw std::runtime_error("no connection types registered!");
    }

    if(i.type_to_constructor.find(type) == i.type_to_constructor.end()) {
        throw std::runtime_error(std::string("cannot create message, no such type (") + type + ")");
    }

    return i.type_to_constructor[type]();
}

TokenData::Ptr MessageFactory::readMessage(const std::string &path)
{
    YAML::Node node;
    if(path.substr(path.size() - Settings::message_extension_compressed.size()) == Settings::message_extension_compressed)
    {
        std::ifstream in (path, std::ios_base::in | std::ios_base::binary);
        boost::iostreams::filtering_istream zipped_in;
        zipped_in.push(boost::iostreams::gzip_decompressor());
        zipped_in.push(in);
        node = YAML::Load(zipped_in);
    }
    else{
        node = YAML::LoadFile(path);
    }
    return MessageSerializer::instance().readYaml(node);
}

void MessageFactory::writeMessage(const std::string &path, const TokenData& msg)
{
    std::ofstream out(path.c_str());

    YAML::Emitter yaml;
    yaml << MessageSerializer::instance().serializeMessage(msg);
    out << yaml.c_str();
}

void MessageFactory::writeMessage(YAML::Emitter &yaml, const TokenData& msg)
{
    yaml << MessageSerializer::instance().serializeMessage(msg);
}

bool MessageFactory::isMessageRegistered(const std::string &type) const
{
    std::map<std::string, Constructor>::const_iterator it = type_to_constructor.find(type);
    return it != type_to_constructor.end();
}

void MessageFactory::registerMessage(std::string type, std::type_index typeindex, Constructor constructor)
{
    MessageFactory& i = instance();

    std::map<std::string, Constructor>::const_iterator it = i.type_to_constructor.find(type);

    apex_assert_hard_msg(it == i.type_to_constructor.end() || i.type_to_type_index.at(type) == typeindex,
                         std::string("there are two or more messages of type '") + type + "' registered");

    i.type_to_constructor.insert(std::make_pair(type, constructor));
    i.type_to_type_index.insert(std::make_pair(type, typeindex));
}
