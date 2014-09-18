#ifndef MESSAGE_FACTORY_H
#define MESSAGE_FACTORY_H

/// COMPONENT
#include <csapex/model/connection_type.h>
#include <csapex/msg/message.h>
#include <csapex/msg/message_traits.h>

/// PROJECT
#include <csapex/utility/singleton.hpp>

/// SYSTEM
#include <map>
#include <string>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <utils_yaml/yamlplus.h>

namespace csapex {

class MessageFactory : public Singleton<MessageFactory>
{
    friend class Singleton<MessageFactory>;

    template <typename M>
    friend class csapex::connection_types::MessageRegistered;

public:
    typedef boost::function<ConnectionType::Ptr()>  Constructor;
    struct Converter
    {
        typedef boost::function<YAML::Node(const csapex::ConnectionType&)> Encoder;
        typedef boost::function<bool(const YAML::Node&, csapex::ConnectionType&)> Decoder;

        Converter(Encoder encode, Decoder decode)
            : encoder(encode), decoder(decode)
        {}

        Encoder encoder;
        Decoder decoder;
    };

    typedef std::runtime_error SerializationError;
    typedef std::runtime_error DeserializationError;

public:
    template <typename M>
    static ConnectionType::Ptr createMessage() {
        return connection_types::makeEmpty<M>();
    }

    static ConnectionType::Ptr createMessage(const std::string& type);

    static ConnectionType::Ptr deserializeMessage(const YAML::Node &node);
    static YAML::Node serializeMessage(const ConnectionType& msg);

    static ConnectionType::Ptr readMessage(const std::string& path);
    static void writeMessage(const std::string& path, const ConnectionType &msg);

    static ConnectionType::Ptr readYaml(const YAML::Node& node);

private:
    template <typename M>
    static void registerMessage() {
        MessageFactory::instance().registerMessage(boost::bind(&MessageFactory::createMessage<M>),
                                                   Converter(boost::bind(&MessageFactory::encode<M>, _1),
                                                             boost::bind(&MessageFactory::decode<M>, _1, _2)));
    }

    template <typename Message>
    static YAML::Node encode(const csapex::ConnectionType& msg) {
        return YAML::convert<Message>::encode(dynamic_cast<const Message&>(msg));
    }

    template <typename Message>
    static bool decode(const YAML::Node& node, csapex::ConnectionType& msg) {
        return YAML::convert<Message>::decode(node, dynamic_cast<Message&>(msg));
    }

private:
    MessageFactory();

    static void registerMessage(Constructor constructor, Converter converter);

private:
    std::map<std::string, Constructor> type_to_constructor;
    std::map<std::string, Converter> type_to_converter;
};
}

#endif // MESSAGE_FACTORY_H
