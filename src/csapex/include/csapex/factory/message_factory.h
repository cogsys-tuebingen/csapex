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
#include <functional>
#include <boost/type_traits.hpp>
#include <iostream>

namespace csapex {

class MessageFactory : public Singleton<MessageFactory>
{
    friend class Singleton<MessageFactory>;

public:
    typedef std::function<ConnectionType::Ptr()>  Constructor;

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

    static ConnectionType::Ptr readMessage(const std::string& path);
    static void writeMessage(const std::string& path, const ConnectionType &msg);

public:
    template <template <typename> class Wrapper, typename M>
    static void registerDirectMessage()
    {
        MessageFactory::instance().registerMessage(connection_types::serializationName< Wrapper<M> >(),
                                                   std::bind(&MessageFactory::createDirectMessage<Wrapper, M>));
    }

    template <typename M>
    static void registerMessage() {
        MessageFactory::instance().registerMessage(connection_types::serializationName<M>(),
                                                   std::bind(&MessageFactory::createMessage<M>));
    }

private:
    MessageFactory();

    static void registerMessage(std::string type, Constructor constructor);

private:
    std::map<std::string, Constructor> type_to_constructor;
};



template <typename T>
struct MessageConstructorRegistered
{
    MessageConstructorRegistered() {
        csapex::MessageFactory::registerMessage<T>();
    }
};

template <template <typename> class Wrapper, typename T>
struct DirectMessageConstructorRegistered
{
    DirectMessageConstructorRegistered() {
        csapex::MessageFactory::registerDirectMessage<Wrapper, T>();
    }
};

}

#endif // MESSAGE_FACTORY_H
