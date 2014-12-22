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
#include <boost/function.hpp>

namespace csapex {

class MessageRendererManager : public Singleton<MessageRendererManager>
{
    friend class Singleton<MessageRendererManager>;

public:
    typedef boost::function<MessageRenderer::Ptr()>  Constructor;

public:
    // TODO: remove singleton -> put this in constructor
    void setPluginLocator(PluginLocatorPtr locator);
    void shutdown();

    MessageRendererPtr createMessageRenderer(const ConnectionTypePtr& message);

private:
    MessageRendererManager();
    ~MessageRendererManager();

    void loadPlugins();

private:
    csapex::PluginLocatorPtr plugin_locator_;

    std::map<const std::type_info*, MessageRendererPtr> renderers;

    PluginManager<MessageRenderer>* manager_;
};

}

#endif // MESSAGE_RENDERER_MANAGER_H

