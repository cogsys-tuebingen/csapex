#ifndef MESSAGE_PROVIDER_MANAGER_H
#define MESSAGE_PROVIDER_MANAGER_H

/// COMPONENT
#include <csapex/message_provider.h>

/// PROJECT
#include <utils_plugin/singleton.hpp>
#include <utils_plugin/plugin_manager.hpp>

/// SYSTEM
#include <map>
#include <string>
#include <boost/function.hpp>

namespace csapex {

class MessageProviderManager : public Singleton<MessageProviderManager>
{
    friend class Singleton<MessageProviderManager>;

public:
    typedef boost::function<MessageProvider::Ptr()>  Constructor;

public:
    static void registerMessageProvider(const std::string& type, Constructor constructor);
    static MessageProvider::Ptr createMessageProvider(const std::string& path);

    std::string supportedTypes();

private:
    MessageProviderManager();
    MessageProvider::Ptr createMessageProviderHelper(const std::string& path);

    void fullReload();

private:
    std::map<std::string, Constructor> classes;

    std::string supported_types_;

    PluginManager<MessageProvider> manager_;
};

}

#endif // MESSAGE_PROVIDER_MANAGER_H
