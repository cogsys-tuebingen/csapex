#ifndef TRACING_LEGEND_H
#define TRACING_LEGEND_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/model/observer.h>
#include <csapex_qt_export.h>

/// SYSTEM
#include <QFrame>
#include <QTableWidget>

namespace csapex
{
class CSAPEX_QT_EXPORT TracingLegend : public QTableWidget, public Observer
{
    Q_OBJECT

public:
    TracingLegend();

    void resizeToFit();

public Q_SLOTS:
    void startTrackingNode(NodeFacadePtr node);
    void stopTrackingNode(NodeFacadePtr node);

    void addNode(NodeFacade* node);
    void removeNode(NodeFacade* node);

    void emitSelection();

Q_SIGNALS:
    void nodeSelectionChanged(QList<NodeFacade*>);

    void nodeAdded(NodeFacade*);
    void nodeRemoved(NodeFacade*);

private:
    std::vector<NodeFacade*> rows_;
};

}  // namespace csapex

#endif  // TRACING_LEGEND_H
