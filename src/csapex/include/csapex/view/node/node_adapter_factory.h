#ifndef NODE_ADAPTER_FACTORY_H
#define NODE_ADAPTER_FACTORY_H

/// COMPONENT
#include <csapex/core/core_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/view/node/node_adapter_builder.h>

/// SYSTEM
#include <map>

namespace csapex
{

class NodeAdapterFactory
{
    friend class WidgetController;

public:
    typedef std::shared_ptr<NodeAdapterFactory> Ptr;

public:
    NodeAdapterFactory(Settings& settings, csapex::PluginLocator* locator);
    ~NodeAdapterFactory();

    NodeAdapterPtr makeNodeAdapter(NodeHandlePtr node, NodeBox* parent);

    void loadPlugins();

private:
    void ensureLoaded();
    void rebuildPrototypes();

protected:
    Settings& settings_;
    csapex::PluginLocator* plugin_locator_;

    PluginManager<NodeAdapterBuilder>* node_adapter_manager_;
    std::map<std::string, NodeAdapterBuilder::Ptr> node_adapter_builders_;
};

}

#endif // NODE_ADAPTER_FACTORY_H
