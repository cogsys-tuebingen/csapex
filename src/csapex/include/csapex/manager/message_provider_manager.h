#ifndef MESSAGE_PROVIDER_MANAGER_H
#define MESSAGE_PROVIDER_MANAGER_H

/// COMPONENT
#include <csapex/msg/message_provider.h>
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <csapex/utility/singleton.hpp>

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
    ~MessageProviderManager();
    MessageProvider::Ptr createMessageProviderHelper(const std::string& path);

    void loadPlugins();

private:
    std::map<std::string, Constructor> classes;

    std::string supported_types_;

    PluginManager<MessageProvider>* manager_;
};

}

#endif // MESSAGE_PROVIDER_MANAGER_H
