/// HEADER
#include <csapex/factory/message_factory.h>

/// COMPONENT
#include <csapex/utility/assert.h>
#include <csapex/utility/yaml_node_builder.h>
#include <csapex/serialization/message_serializer.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/core/settings.h>

/// SYSTEM
#include <fstream>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <iostream>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/version.hpp>

#if (BOOST_VERSION / 100000) >= 1 && (BOOST_VERSION / 100 % 1000) >= 54
namespace bf3 = boost::filesystem;
#else
namespace bf3 = boost::filesystem3;
#endif

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

    if (i.type_to_constructor.empty()) {
        throw std::runtime_error("no connection types registered!");
    }

    if (i.type_to_constructor.find(type) == i.type_to_constructor.end()) {
        throw std::runtime_error(std::string("cannot create message, no such type (") + type + ")");
    }

    return i.type_to_constructor[type]();
}

TokenData::Ptr MessageFactory::readFile(const std::string& file)
{
    std::string ext = file.substr(file.rfind("."));

    if (ext == Settings::message_extension_binary) {
        return readBinaryFile(file);
    } else {
        return readYamlFile(file);
    }

    return nullptr;
}

int MessageFactory::writeFile(const std::string& path, const std::string& base, const int suffix, const TokenData& msg, serialization::Format format)
{
    int next_free_suffix = suffix;

    // make sure the target directory exists
    bf3::path dir(path);
    if (!bf3::exists(dir)) {
        bf3::create_directories(dir);
    }

    if (format == serialization::Format::NATIVE) {
        msg.writeNative(path, base, std::to_string(next_free_suffix));
        return next_free_suffix + 1;
    }

    // generate a unique file name
    std::string file_name;
    bool file_exists = false;
    do {
        std::stringstream file_s;
        switch (format) {
            case serialization::Format::APEX_YAML:
                file_s << path << "/" << base << "_" << next_free_suffix << Settings::message_extension;
                break;
            case serialization::Format::APEX_BINARY:
                file_s << path << "/" << base << "_" << next_free_suffix << Settings::message_extension_binary;
                break;
            case serialization::Format::NATIVE:
                break;
        }
        file_name = file_s.str();

        if (bf3::exists((file_name))) {
            ++next_free_suffix;
        }
    } while (file_exists);

    switch (format) {
        case serialization::Format::APEX_YAML:
            writeYamlFile(file_name, msg);
            break;
        case serialization::Format::APEX_BINARY:
            writeBinaryFile(file_name, msg);
            break;
        case serialization::Format::NATIVE:
            // DO NOTHING
            break;
    }

    return next_free_suffix;
}

TokenData::Ptr MessageFactory::readYamlFile(const std::string& path)
{
    YAML::Node node;
    if (path.substr(path.size() - Settings::message_extension_compressed.size()) == Settings::message_extension_compressed) {
        std::ifstream in(path, std::ios_base::in | std::ios_base::binary);
        boost::iostreams::filtering_istream zipped_in;
        zipped_in.push(boost::iostreams::gzip_decompressor());
        zipped_in.push(in);
        node = YAML::Load(zipped_in);
    } else {
        node = YAML::LoadFile(path);
    }
    return MessageSerializer::instance().readYaml(node);
}

void MessageFactory::writeYamlFile(const std::string& path, const TokenData& msg)
{
    std::ofstream out(path.c_str());

    YAML::Emitter yaml;
    yaml << MessageSerializer::instance().serializeYamlMessage(msg);
    out << yaml.c_str();
}

TokenData::Ptr MessageFactory::readBinaryFile(const std::string& path)
{
    auto file = std::fopen(path.c_str(), "rb");
    if (!file) {
        throw std::runtime_error("cannot open file " + path + " for reading");
    }

    std::fseek(file, 0, SEEK_END);
    std::size_t n = std::ftell(file);
    std::rewind(file);

    SerializationBuffer buffer;
    buffer.resize(n);

    std::fread(buffer.data(), 1, n, file);

    return MessageSerializer::instance().deserializeBinaryMessage(buffer);
}

void MessageFactory::writeBinaryFile(const std::string& path, const TokenData& msg)
{
    auto file = std::fopen(path.c_str(), "wb");
    if (!file) {
        throw std::runtime_error("cannot open file " + path + " for writing");
    }

    SerializationBuffer buffer;
    MessageSerializer::serializeBinaryMessage(msg, buffer);
    buffer.finalize();

    std::fwrite(buffer.data(), sizeof(uint8_t), buffer.size(), file);
}

bool MessageFactory::isMessageRegistered(const std::string& type) const
{
    std::map<std::string, Constructor>::const_iterator it = type_to_constructor.find(type);
    return it != type_to_constructor.end();
}

void MessageFactory::registerMessage(std::string type, std::type_index typeindex, Constructor constructor)
{
    MessageFactory& i = instance();

    std::map<std::string, Constructor>::const_iterator it = i.type_to_constructor.find(type);

    apex_assert_hard_msg(it == i.type_to_constructor.end() || i.type_to_type_index.at(type) == typeindex, std::string("there are two or more messages of type '") + type + "' registered");

    i.type_to_constructor.insert(std::make_pair(type, constructor));
    i.type_to_type_index.insert(std::make_pair(type, typeindex));
}
