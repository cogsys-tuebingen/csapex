#ifndef NODE_FACTORY_H
#define NODE_FACTORY_H

/// COMPONENT
#include <csapex/model/node_constructor.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <vector>
#include <boost/signals2.hpp>

namespace csapex
{

class NodeFactory
{
public:
    typedef boost::shared_ptr<NodeFactory> Ptr;

public:
    NodeFactory(Settings& settings);
    ~NodeFactory();

    void loadPlugins();

public:
    void registerNodeType(NodeConstructor::Ptr provider, bool suppress_signals = false);

    bool isValidType(const std::string& type) const;

    NodeConstructorPtr getConstructor(const std::string& type);
    std::vector<NodeConstructorPtr> getConstructors();

    NodePtr makeNode(const std::string& type, const UUID& uuid);
    NodePtr makeNode(const std::string& type, const UUID& uuid, NodeStatePtr state);

    std::map<TagPtr, std::vector<NodeConstructor::Ptr> > getTagMap();

public:
    boost::signals2::signal<void(const std::string&)> loaded;
    boost::signals2::signal<void()> new_node_type;

protected:
    void ensureLoaded();
    void rebuildPrototypes();
    void rebuildMap();

    NodePtr makeSingleNode(NodeConstructor::Ptr content, const UUID& uuid);

protected:
    Settings& settings_;

    std::map<TagPtr, std::vector<NodeConstructor::Ptr> > tag_map_;
    std::vector<NodeConstructor::Ptr> constructors_;

    PluginManager<Node>* node_manager_;

    bool tag_map_has_to_be_rebuilt_;
};

}

#endif // NODE_FACTORY_H
