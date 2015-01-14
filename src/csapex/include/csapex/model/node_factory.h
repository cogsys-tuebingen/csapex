#ifndef NODE_FACTORY_H
#define NODE_FACTORY_H

/// COMPONENT
#include <csapex/model/node_constructor.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <vector>
#include <boost/signals2.hpp>
#include <unordered_map>

namespace csapex
{

class NodeFactory : private boost::noncopyable
{
public:
    typedef std::shared_ptr<NodeFactory> Ptr;

    boost::signals2::signal<void(const UUID&)> unload_request;
    boost::signals2::signal<void(const UUID&)> reload_request;

public:
    NodeFactory(Settings& settings, PluginLocator *locator);
    ~NodeFactory();

    void loadPlugins();

public:
    void registerNodeType(NodeConstructor::Ptr provider, bool suppress_signals = false);

    bool isValidType(const std::string& type) const;

    NodeConstructorPtr getConstructor(const std::string& type);
    std::vector<NodeConstructorPtr> getConstructors();

    NodeWorkerPtr makeNode(const std::string& type, const UUID& uuid);
    NodeWorkerPtr makeNode(const std::string& type, const UUID& uuid, NodeStatePtr state);

    std::map<TagPtr, std::vector<NodeConstructor::Ptr> > getTagMap();

public:
    boost::signals2::signal<void(const std::string&)> loaded;
    boost::signals2::signal<void()> new_node_type;

protected:
    void unloadNode(NodeConstructorPtr p, UUID uuid);
    void reloadNode(NodeConstructorPtr p, UUID uuid);

    void ensureLoaded();
    void rebuildPrototypes();
    void rebuildMap();

    NodeWorkerPtr makeSingleNode(NodeConstructor::Ptr content, const UUID& uuid);

protected:
    Settings& settings_;
    csapex::PluginLocator* plugin_locator_;

    std::map<TagPtr, std::vector<NodeConstructor::Ptr> > tag_map_;
    std::vector<NodeConstructor::Ptr> constructors_;

    std::unordered_map<UUID, boost::signals2::connection, UUID::Hasher> reload_connections_;

    PluginManager<Node>* node_manager_;

    bool tag_map_has_to_be_rebuilt_;
};

}

#endif // NODE_FACTORY_H
