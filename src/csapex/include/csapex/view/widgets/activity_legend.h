#ifndef ACTIVITY_LEGEND_H
#define ACTIVITY_LEGEND_H

/// COMPONENT
#include <csapex/model/node_worker.h>

/// SYSTEM
#include <QFrame>
#include <QTableWidget>

namespace csapex
{

class ActivityLegend : public QTableWidget
{
    Q_OBJECT

public:
    ActivityLegend();

    void resizeToFit();

public Q_SLOTS:
    void startTrackingNode(NodeWorkerPtr node);
    void stopTrackingNode(NodeWorkerPtr node);

    void addNode(NodeWorker* node);
    void removeNode(NodeWorker* node);

    void emitSelection();

Q_SIGNALS:
    void nodeSelectionChanged(QList<NodeWorker*>);

    void nodeAdded(NodeWorker*);
    void nodeRemoved(NodeWorker*);


private:
    std::vector<NodeWorker*> rows_;
};

}

#endif // ACTIVITY_LEGEND_H

