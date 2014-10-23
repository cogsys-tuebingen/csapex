#ifndef NODE_ADAPTER_FACTORY_H
#define NODE_ADAPTER_FACTORY_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/view/node_adapter_builder.h>

/// SYSTEM
#include <map>

namespace csapex
{

class NodeAdapterFactory
{
    friend class WidgetController;

public:
    typedef boost::shared_ptr<NodeAdapterFactory> Ptr;

public:
    NodeAdapterFactory(Settings& settings);
    ~NodeAdapterFactory();

    NodeAdapterPtr makeNodeAdapter(NodeWorker* node, WidgetController* widget_controller);

    void loadPlugins();

private:
    void ensureLoaded();
    void rebuildPrototypes();

protected:
    Settings& settings_;

    PluginManager<NodeAdapterBuilder>* node_adapter_manager_;
    std::map<std::string, NodeAdapterBuilder::Ptr> node_adapter_builders_;
};

}

#endif // NODE_ADAPTER_FACTORY_H
