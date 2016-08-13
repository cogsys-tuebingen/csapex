#ifndef NODE_ADAPTER_FACTORY_H
#define NODE_ADAPTER_FACTORY_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/core/core_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/view/node/node_adapter_builder.h>

/// SYSTEM
#include <map>

namespace csapex
{

class CSAPEX_QT_EXPORT NodeAdapterFactory
{
public:
    typedef std::shared_ptr<NodeAdapterFactory> Ptr;

public:
    NodeAdapterFactory(Settings& settings, csapex::PluginLocator* locator);
    ~NodeAdapterFactory();

    NodeAdapterPtr makeNodeAdapter(NodeHandlePtr node, NodeBox* parent);
    bool hasAdapter(const std::string& type) const;

    void loadPlugins();

private:
    void ensureLoaded();
//    void rebuildPrototypes();

protected:
    Settings& settings_;
    csapex::PluginLocator* plugin_locator_;

    PluginManager<NodeAdapterBuilder>* node_adapter_manager_;
    std::map<std::string, NodeAdapterBuilder::Ptr> node_adapter_builders_;
};

}

#endif // NODE_ADAPTER_FACTORY_H
