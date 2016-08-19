/// HEADER
#include <csapex/manager/message_renderer_manager.h>

/// COMPONENT
#include <csapex/plugin/plugin_manager.hpp>
#include <csapex/core/settings.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/apex_message_provider.h>

using namespace csapex;

MessageRendererManager::MessageRendererManager()
    : manager_(new PluginManager<MessageRenderer>("csapex::MessageRenderer"))
{
}

MessageRendererManager::~MessageRendererManager()
{
}

void MessageRendererManager::setPluginLocator(PluginLocatorPtr locator)
{
    plugin_locator_ = locator;
}

void MessageRendererManager::shutdown()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
	plugin_locator_.reset();
    renderers.clear();
}

void MessageRendererManager::loadPlugins()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    apex_assert_hard(manager_);

    if(!manager_->pluginsLoaded()) {
        manager_->load(plugin_locator_.get());
    }

    renderers.clear();

    for(const auto& pair : manager_->getConstructors()) {
        try {
            MessageRenderer::Ptr renderer(pair.second());
            renderers[renderer->messageType()] = renderer;

        } catch(const std::exception& e) {
            std::cerr << "cannot load message renderer " << pair.first << ": " << typeid(e).name() << ", what=" << e.what() << std::endl;
        }
    }
}

MessageRendererPtr MessageRendererManager::createMessageRenderer(const TokenDataConstPtr& message)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    if(!manager_) {
        return nullptr;
    }

    if(!manager_->pluginsLoaded() || renderers.empty()) {
        loadPlugins();
    }
    if(renderers.empty()) {
        throw std::runtime_error("no message renderers registered!");
    }

    const TokenData& m = *message;
    try {
        auto pos = renderers.find(std::type_index(typeid(m)));
        if(pos != renderers.end()) {
            return pos->second;
        } else {
            return nullptr;
        }
    } catch(const std::exception& /*e*/) {
        throw std::runtime_error(std::string("cannot create message renderer for ") + type2name(typeid(m)));
    }
}
