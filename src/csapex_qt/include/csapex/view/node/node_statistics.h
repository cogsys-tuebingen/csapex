#ifndef NODE_STATISTICS_H
#define NODE_STATISTICS_H

/// COMPONENT
#include <csapex_qt_export.h>

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/factory/factory_fwd.h>

/// SYSTEM
#include <QTreeWidgetItem>

namespace csapex
{
class CSAPEX_QT_EXPORT NodeStatistics
{
public:
    NodeStatistics(NodeFacade* node);
    QTreeWidgetItem* createDebugInformation(NodeFactory* node_factory) const;

private:
    QTreeWidgetItem* createDebugInformationConnector(const csapex::ConnectorDescription& connector) const;

private:
    NodeFacade* node_facade_;
};
}  // namespace csapex

#endif  // NODE_STATISTICS_H
