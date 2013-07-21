#ifndef CONNECTION_TYPE_MANAGER_H
#define CONNECTION_TYPE_MANAGER_H

/// COMPONENT
#include "connection_type.h"

/// SYSTEM
#include <map>
#include <string>
#include <boost/function.hpp>

class ConnectionTypeManager
{
public:
    typedef boost::function<ConnectionType::Ptr()>  Constructor;

public:
    static void registerMessage(const std::string& type, Constructor constructor);
    static ConnectionType::Ptr createMessage(const std::string& type);
    static ConnectionTypeManager& instance();

private:
    ConnectionTypeManager();

private:
    std::map<std::string, Constructor> classes;
};

#endif // CONNECTION_TYPE_MANAGER_H
