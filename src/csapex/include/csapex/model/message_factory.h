#ifndef MESSAGE_FACTORY_H
#define MESSAGE_FACTORY_H

/// COMPONENT
#include <csapex/model/connection_type.h>

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

public:
    typedef boost::function<ConnectionType::Ptr()>  Constructor;

    typedef std::runtime_error DeserializationError;

public:
    template <typename M>
    static void registerMessage() {
        registerMessage(boost::bind(&M::make));
    }

    static ConnectionType::Ptr createMessage(const std::string& type);

    static ConnectionType::Ptr readMessage(const std::string& path);
    static void writeMessage(const std::string& path, const ConnectionType::Ptr msg);

    static ConnectionType::Ptr readYaml(const YAML::Node& node);

private:
    MessageFactory();

    static void registerMessage(Constructor constructor);

private:
    std::map<std::string, Constructor> classes;
};

}

#endif // MESSAGE_FACTORY_H
