#ifndef CONNECTION_TYPE_MANAGER_H
#define CONNECTION_TYPE_MANAGER_H

/// COMPONENT
#include <csapex/connection_type.h>

/// PROJECT
#include <utils_plugin/singleton.hpp>

/// SYSTEM
#include <map>
#include <string>
#include <boost/function.hpp>

namespace csapex {

class ConnectionTypeManager : public Singleton<ConnectionTypeManager>
{
    friend class Singleton<ConnectionTypeManager>;

public:
    typedef boost::function<ConnectionType::Ptr()>  Constructor;

public:
    static void registerMessage(const std::string& type, Constructor constructor);
    static ConnectionType::Ptr createMessage(const std::string& type);

private:
    ConnectionTypeManager();

private:
    std::map<std::string, Constructor> classes;
};

}

#endif // CONNECTION_TYPE_MANAGER_H
