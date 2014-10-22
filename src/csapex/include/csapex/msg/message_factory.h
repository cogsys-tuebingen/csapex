#ifndef MESSAGE_FACTORY_H
#define MESSAGE_FACTORY_H

/// COMPONENT
#include <csapex/model/connection_type.h>
#include <csapex/msg/message.h>
#include <csapex/msg/message_traits.h>

/// PROJECT
#include <csapex/utility/singleton.hpp>
#include <csapex/utility/tmp.hpp>

/// SYSTEM
#include <map>
#include <string>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/type_traits.hpp>
#include <iostream>

HAS_MEM_FUNC(encode, has_yaml_implementation);

namespace csapex {

class MessageFactory : public Singleton<MessageFactory>
{
    friend class Singleton<MessageFactory>;

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
    template <template <typename> class Wrapper,typename M>
    static ConnectionType::Ptr createDirectMessage() {
        return connection_types::makeEmptyMessage< Wrapper<M> >();
    }

    static ConnectionType::Ptr createMessage(const std::string& type);

    static ConnectionType::Ptr deserializeMessage(const YAML::Node &node);
    static YAML::Node serializeMessage(const ConnectionType& msg);

    static ConnectionType::Ptr readMessage(const std::string& path);
    static void writeMessage(const std::string& path, const ConnectionType &msg);

    static ConnectionType::Ptr readYaml(const YAML::Node& node);

public:
    template <template <typename> class Wrapper, typename M>
    static void registerDirectMessage()
    {
        registerDirectMessageImpl<Wrapper, M>();
    }

    template <typename M>
    static void registerMessage() {
        MessageFactory::instance().registerMessage(connection_types::name<M>(),
                                                   boost::bind(&MessageFactory::createMessage<M>),
                                                   Converter(boost::bind(&MessageFactory::encode<M>, _1),
                                                             boost::bind(&MessageFactory::decode<M>, _1, _2)));
    }

private:
    template <template <typename> class Wrapper, typename M>
    struct HasYaml {
        typedef typename boost::type_traits::ice_and<
        boost::is_class<M>::value,
        has_yaml_implementation< YAML::convert<M>, YAML::Node(YAML::convert<M>::*)(const M&) >::value
        > type;
    };

    template <template <typename> class Wrapper, typename M>
    static void registerDirectMessageImpl(typename boost::disable_if< typename HasYaml<Wrapper,M>::type >::type* = 0)
    {
        std::cerr << "Using a direct message that is not serializable: " << type2name(typeid(M)) << std::endl;
    }
    template <template <typename> class Wrapper, typename M>
    static void registerDirectMessageImpl(typename boost::enable_if< typename HasYaml<Wrapper,M>::type >::type* = 0)
    {
        MessageFactory::instance().registerMessage(connection_types::name< Wrapper<M> >(),
                                                   boost::bind(&MessageFactory::createDirectMessage<Wrapper, M>),
                                                   Converter(boost::bind(&MessageFactory::encodeDirectMessage<Wrapper, M>, _1),
                                                             boost::bind(&MessageFactory::decodeDirectMessage<Wrapper, M>, _1, _2)));
    }

    template <typename Message>
    static YAML::Node encode(const csapex::ConnectionType& msg) {
        return YAML::convert<Message>::encode(dynamic_cast<const Message&>(msg));
    }
    template <typename Message>
    static bool decode(const YAML::Node& node, csapex::ConnectionType& msg) {
        return YAML::convert<Message>::decode(node, dynamic_cast<Message&>(msg));
    }


    template <template <typename> class Wrapper, typename Message>
    static YAML::Node encodeDirectMessage(const csapex::ConnectionType& msg) {
        typedef Wrapper<Message> Implementation;
        const Implementation& impl = dynamic_cast<const Implementation&>(msg);

        return YAML::convert<Message>::encode(impl);
    }
    template <template <typename> class Wrapper, typename Message>
    static bool decodeDirectMessage(const YAML::Node& node, csapex::ConnectionType& msg) {
        typedef Wrapper<Message> Implementation;
        Implementation& impl = dynamic_cast<Implementation&>(msg);

        return YAML::convert<Message>::decode(node, impl);
    }

private:
    MessageFactory();

    static void registerMessage(std::string type, Constructor constructor, Converter converter);

private:
    std::map<std::string, Constructor> type_to_constructor;
    std::map<std::string, Converter> type_to_converter;
};



template <typename T>
struct MessageRegistered
{
    MessageRegistered() {
        csapex::MessageFactory::registerMessage<T>();
    }
};

}

#endif // MESSAGE_FACTORY_H
