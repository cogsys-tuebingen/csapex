#ifndef CONNECTION_TYPE_MANAGER_H
#define CONNECTION_TYPE_MANAGER_H

/// COMPONENT
#include <csapex/model/connection_type.h>

/// PROJECT
#include <utils_plugin/singleton.hpp>

/// SYSTEM
#include <map>
#include <string>
#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace csapex {

class ConnectionTypeManager : public Singleton<ConnectionTypeManager>
{
    friend class Singleton<ConnectionTypeManager>;

public:
    typedef boost::function<ConnectionType::Ptr()>  Constructor;

public:
    template <typename M>
    static void registerMessage(const std::string& type) {
        registerMessage(type, boost::bind(&M::make));
    }

    static ConnectionType::Ptr createMessage(const std::string& type);

private:
    ConnectionTypeManager();

    static void registerMessage(const std::string& type, Constructor constructor);

private:
    std::map<std::string, Constructor> classes;
};

}

#endif // CONNECTION_TYPE_MANAGER_H
