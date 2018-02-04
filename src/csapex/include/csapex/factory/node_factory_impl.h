#ifndef NODE_FACTORY_IMPL_H
#define NODE_FACTORY_IMPL_H

/// COMPONENT
#include <csapex/factory/node_factory.h>
#include <csapex/model/node_constructor.h>
#include <csapex/utility/uuid.h>

/// PROJECT
#include <csapex/model/notifier.h>
#include <csapex/core/core_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/csapex_export.h>
#include <csapex/model/execution_type.h>

/// SYSTEM
#include <vector>
#include <csapex/utility/slim_signal.h>
#include <unordered_map>

class TiXmlElement;

namespace csapex
{

class CSAPEX_EXPORT NodeFactoryImplementation : public NodeFactory
{
public:
    NodeFactoryImplementation(Settings& settings, PluginLocator *locator);

    void loadPlugins();
    void shutdown();

    void registerNodeType(NodeConstructor::Ptr provider, bool suppress_signals = false);

    NodeFacadeImplementationPtr makeNode(const std::string& type,
                                         const UUID& uuid,
                                         const UUIDProviderPtr &uuid_provider,
                                         const ExecutionType exec_type = ExecutionType::AUTO);
    NodeFacadeImplementationPtr makeNode(const std::string& type,
                                         const UUID& uuid,
                                         const UUIDProviderPtr &uuid_provider,
                                         const ExecutionType exec_type,
                                         NodeStatePtr state);

    NodeFacadeImplementationPtr makeGraph(const UUID& uuid,
                                          const UUIDProviderPtr &uuid_provider);
    NodeFacadeImplementationPtr makeGraph(const UUID& uuid,
                                          const UUIDProviderPtr &uuid_provider,
                                          NodeStatePtr state);


public:
    slim_signal::Signal<void(const std::string&)> loaded;
    slim_signal::Signal<void()> new_node_type;
    slim_signal::Signal<void(NodeFacadePtr)> node_constructed;
    slim_signal::Signal<void(const std::string& file, const TiXmlElement* document)> manifest_loaded;

protected:
    void ensureLoaded() override;
    void rebuildPrototypes();
    void rebuildMap();

protected:
    Settings &settings_;
    csapex::PluginLocator* plugin_locator_;

    std::shared_ptr<PluginManager<Node>> node_manager_;

    bool tag_map_has_to_be_rebuilt_;
};

}

#endif // NODE_FACTORY_IMPL_H
