#ifndef MESSAGE_RENDERER_MANAGER_H
#define MESSAGE_RENDERER_MANAGER_H

/// COMPONENT
#include <csapex_core/csapex_core_export.h>
#include <csapex/msg/message_renderer.h>

/// PROJECT
#include <csapex/msg/msg_fwd.h>
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/utility/singleton.hpp>

/// SYSTEM
#include <map>
#include <string>
#include <functional>
#include <mutex>
#include <typeindex>

namespace csapex
{
class CSAPEX_CORE_EXPORT MessageRendererManager : public Singleton<MessageRendererManager>
{
    friend class Singleton<MessageRendererManager>;

public:
    typedef std::function<MessageRenderer::Ptr()> Constructor;

public:
    // TODO: remove singleton -> put this in constructor
    void setPluginLocator(PluginLocatorPtr locator);
    void shutdown();

    MessageRendererPtr createMessageRenderer(const TokenDataConstPtr& message);

private:
    MessageRendererManager();
    ~MessageRendererManager();

    void loadPlugins();

private:
    std::recursive_mutex mutex_;

    csapex::PluginLocatorPtr plugin_locator_;

    std::map<std::type_index, MessageRendererPtr> renderers;

    std::unique_ptr<PluginManager<MessageRenderer>> manager_;
};

}  // namespace csapex

#endif  // MESSAGE_RENDERER_MANAGER_H
