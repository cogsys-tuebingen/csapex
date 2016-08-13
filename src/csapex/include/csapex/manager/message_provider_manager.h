#ifndef MESSAGE_PROVIDER_MANAGER_H
#define MESSAGE_PROVIDER_MANAGER_H

/// COMPONENT
#include <csapex/msg/message_provider.h>

/// PROJECT
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/utility/singleton.hpp>

/// SYSTEM
#include <map>
#include <string>
#include <functional>

namespace csapex {

class CSAPEX_EXPORT MessageProviderManager : public Singleton<MessageProviderManager>
{
    friend class Singleton<MessageProviderManager>;

public:
    typedef std::function<MessageProvider::Ptr()>  Constructor;

public:    
    static void registerMessageProvider(const std::string& type, Constructor constructor);
    static MessageProvider::Ptr createMessageProvider(const std::string& path);

    std::string supportedTypes();

    // TODO: remove singleton -> put this in constructor
    void setPluginLocator(PluginLocatorPtr locator);
    void shutdown();

private:
    MessageProviderManager();
    ~MessageProviderManager();
    MessageProvider::Ptr createMessageProviderHelper(const std::string& path);

    void loadPlugins();

private:
    csapex::PluginLocatorPtr plugin_locator_;

    std::map<std::string, Constructor> classes;

    std::string supported_types_;

    PluginManager<MessageProvider>* manager_;
};

}

#endif // MESSAGE_PROVIDER_MANAGER_H
