#ifndef MESSAGE_RENDERER_MANAGER_H
#define MESSAGE_RENDERER_MANAGER_H

/// COMPONENT
#include <csapex/msg/message_renderer.h>
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <csapex/utility/singleton.hpp>

/// SYSTEM
#include <map>
#include <string>
#include <functional>
#include <mutex>
#include <typeindex>

namespace csapex {

class MessageRendererManager : public Singleton<MessageRendererManager>
{
    friend class Singleton<MessageRendererManager>;

public:
    typedef std::function<MessageRenderer::Ptr()>  Constructor;

public:
    // TODO: remove singleton -> put this in constructor
    void setPluginLocator(PluginLocatorPtr locator);
    void shutdown();

    MessageRendererPtr createMessageRenderer(const ConnectionTypeConstPtr &message);

private:
    MessageRendererManager();
    ~MessageRendererManager();

    void loadPlugins();

private:
    std::recursive_mutex mutex_;

    csapex::PluginLocatorPtr plugin_locator_;

    std::map<std::type_index, MessageRendererPtr> renderers;

    PluginManager<MessageRenderer>* manager_;
};

}

#endif // MESSAGE_RENDERER_MANAGER_H

