#ifndef NODE_STATISTICS_H
#define NODE_STATISTICS_H

/// PROJECT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QTreeWidgetItem>

namespace csapex
{
class NodeStatistics
{
public:
    NodeStatistics(Node* node);
    QTreeWidgetItem* createDebugInformation() const;

private:
    QTreeWidgetItem * createDebugInformationConnector(Connectable *connector) const;

private:
    Node* node_;
};
}

#endif // NODE_STATISTICS_H
