#ifndef NODE_FACTORY_H
#define NODE_FACTORY_H

/// COMPONENT
#include <csapex/model/node_constructor.h>
#include <csapex/csapex_fwd.h>
#include <csapex/view/node_adapter_builder.h>

/// SYSTEM
#include <QLayout>
#include <QMenu>
#include <vector>
#include <QTreeWidget>
#include <boost/signals2.hpp>
#include <set>

namespace csapex
{

class NodeFactory
{
    /// TODO: no friends
    friend class DesignerIO;
    friend class GraphIO;
    friend class WidgetController;

public:
    typedef boost::shared_ptr<NodeFactory> Ptr;

public:
    NodeFactory(Settings& settings);
    ~NodeFactory();

    virtual void reload();

public:
    void register_box_type(NodeConstructor::Ptr provider, bool suppress_signals = false);

    bool isValidType(const std::string& type) const;

    NodeConstructorPtr getConstructor(const std::string& type);
    NodePtr makeNode(const std::string& type, const UUID& uuid);
    NodePtr makeNode(const std::string& type, const UUID& uuid, NodeStatePtr state);

    // TODO: extract
    void insertAvailableNodeTypes(QMenu* menu);
    void insertAvailableNodeTypes(QTreeWidget *tree);
    QAbstractItemModel *listAvailableNodeTypes();

    void stop();

public:
    boost::signals2::signal<void(const std::string&)> loaded;
    boost::signals2::signal<void()> new_box_type;

protected:
    void ensureLoaded();
    void rebuildPrototypes();
    void rebuildMap();

    NodePtr makeSingleNode(NodeConstructor::Ptr content, const UUID& uuid);

protected:
    Settings& settings_;

    std::map<TagPtr, std::vector<NodeConstructor::Ptr> > tag_map_;
    std::set<TagPtr> tags_;

    PluginManager<Node>* node_manager_;
    std::vector<NodeConstructor::Ptr> available_elements_prototypes;

    // TODO: extract to NodeAdapterFactory!
    PluginManager<NodeAdapterBuilder>* node_adapter_manager_;
    std::map<std::string, NodeAdapterBuilder::Ptr> node_adapter_builders_;

    bool dirty_;
};

}

#endif // NODE_FACTORY_H
