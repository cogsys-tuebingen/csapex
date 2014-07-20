#ifndef BOX_MANAGER_H
#define BOX_MANAGER_H

/// COMPONENT
#include <csapex/model/node_constructor.h>
#include <csapex/csapex_fwd.h>
#include <csapex/view/node_adapter_builder.h>

/// PROJECT
#include <csapex/utility/singleton.hpp>

/// SYSTEM
#include <QLayout>
#include <QMenu>
#include <vector>
#include <QTreeWidget>
#include <boost/signals2.hpp>
#include <set>

namespace csapex
{

/// TODO: Split into NodeFactory, BoxFactory
/// TODO: Eliminate Singleton
class BoxManager : public Singleton<BoxManager>
{
    friend class Singleton<BoxManager>;
    friend class DesignerIO;
    friend class GraphIO;

public:
    virtual void reload();
    ~BoxManager();

public:
    void register_box_type(NodeConstructor::Ptr provider, bool suppress_signals = false);

    bool isValidType(const std::string& type) const;

    void startPlacingBox(QWidget *parent, const std::string& type, WidgetController *widget_ctrl, NodeStatePtr state, const QPoint &offset = QPoint(0,0));

    NodeConstructorPtr getConstructor(const std::string& type);
    NodePtr makeNode(const std::string& type, const UUID& uuid);
    NodeBox* makeBox(NodePtr node, WidgetController *widget_ctrl);

    NodeConstructor::Ptr getSelector(const std::string& type);

    void insertAvailableNodeTypes(QMenu* menu);
    void insertAvailableNodeTypes(QTreeWidget *tree);
    QAbstractItemModel *listAvailableNodeTypes();

    void stop();

    void setStyleSheet(const QString& str);


public:
    boost::signals2::signal<void(const std::string&)> loaded;
    boost::signals2::signal<void()> new_box_type;

    // TODO: make private and constructor parameter one no longer singleton
    Settings* settings_;

protected:
    BoxManager();
    BoxManager(const BoxManager& copy);
    BoxManager& operator = (const BoxManager& assign);

    void ensureLoaded();
    void rebuildPrototypes();
    void rebuildMap();

    NodePtr makeSingleNode(NodeConstructor::Ptr content, const UUID& uuid);

protected:
    std::vector<NodeConstructor::Ptr> available_elements_prototypes;
    std::map<std::string, NodeAdapterBuilder::Ptr> node_adapter_builders_;

    std::map<TagPtr, std::vector<NodeConstructor::Ptr> > tag_map_;
    std::set<TagPtr> tags_;

    PluginManager<Node>* node_manager_;
    PluginManager<NodeAdapterBuilder>* node_adapter_manager_;

    bool dirty_;
    QString style_sheet_;
};

}

#endif // BOX_MANAGER_H
