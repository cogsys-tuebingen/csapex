/// HEADER
#include "connection_type_manager.h"

ConnectionTypeManager::ConnectionTypeManager()
{
}

ConnectionTypeManager& ConnectionTypeManager::instance()
{
    static ConnectionTypeManager instance;
    return instance;
}

ConnectionType::Ptr ConnectionTypeManager::createMessage(const std::string& type)
{
    if(instance().classes.empty()) {
        std::cerr << "no connection types registered!" << std::endl;
        throw;
    }

    return instance().classes[type]();
}

void ConnectionTypeManager::registerMessage(const std::string &type, Constructor constructor)
{
    instance().classes[type] = constructor;
}
