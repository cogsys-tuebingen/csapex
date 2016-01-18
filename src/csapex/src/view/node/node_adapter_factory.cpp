/// HEADER
#include <csapex/view/node/node_adapter_factory.h>

/// COMPONENT
#include <csapex/model/node_handle.h>
#include <csapex/view/node/node_adapter.h>
#include <csapex/view/node/default_node_adapter.h>
#include <csapex/plugin/plugin_manager.hpp>

/// SYSTEM
#include <qglobal.h>

using namespace csapex;

NodeAdapterFactory::NodeAdapterFactory(Settings &settings, PluginLocator *locator)
    : settings_(settings), plugin_locator_(locator),
      node_adapter_manager_(new PluginManager<NodeAdapterBuilder> ("csapex::NodeAdapterBuilder"))
{
}


NodeAdapterFactory::~NodeAdapterFactory()
{
    node_adapter_builders_.clear();

    delete node_adapter_manager_;
    node_adapter_manager_ = nullptr;
}

NodeAdapter::Ptr NodeAdapterFactory::makeNodeAdapter(NodeHandlePtr node, NodeBox* parent)
{
    std::string type = node->getType();
    if(node_adapter_builders_.find(type) != node_adapter_builders_.end()) {
        return node_adapter_builders_[type]->build(node, parent);
    } else {
        return NodeAdapter::Ptr(new DefaultNodeAdapter(node, parent));
    }
}

void NodeAdapterFactory::loadPlugins()
{
    if(node_adapter_builders_.empty()) {
        ensureLoaded();
        rebuildPrototypes();
    }
}

void NodeAdapterFactory::rebuildPrototypes()
{
    for(const auto& p : node_adapter_manager_->getConstructors()) {
        const PluginConstructor<NodeAdapterBuilder>& constructor = p.second;

        try {
            NodeAdapterBuilder::Ptr builder = constructor.construct();
            node_adapter_builders_[builder->getWrappedType()] = builder;

        } catch(const std::exception& e) {
            std::cerr << "adapter " << p.first << " cannot be built: " << e.what() << std::endl;
        }
    }
}

void NodeAdapterFactory::ensureLoaded()
{
    if(!node_adapter_manager_->pluginsLoaded()) {
        node_adapter_manager_->load(plugin_locator_);

        rebuildPrototypes();
    }
}
