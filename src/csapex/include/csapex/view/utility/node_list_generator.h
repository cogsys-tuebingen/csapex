#ifndef NODE_LIST_GENERATOR_H
#define NODE_LIST_GENERATOR_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/factory/factory_fwd.h>
#include <csapex/view/node/node_adapter_factory.h>

/// SYSTEM
#include <QMenu>
#include <QTreeWidget>
#include <QStandardItemModel>

namespace csapex
{

class CSAPEX_QT_EXPORT NodeListGenerator
{
public:
    NodeListGenerator(NodeFactory &node_factory, NodeAdapterFactory& adapter_factory);

    void insertAvailableNodeTypes(QMenu* menu);
    void insertAvailableNodeTypes(QTreeWidget *tree);
    void listAvailableNodeTypes(QStandardItemModel *model);

private:
    NodeFactory& node_factory_;
    NodeAdapterFactory& adapter_factory_;
};

}

#endif // NODE_LIST_GENERATOR_H
