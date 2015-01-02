#ifndef ACTIVITY_LEGEND_H
#define ACTIVITY_LEGEND_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
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
    void addNode(NodeWorkerPtr node);
    void removeNode(NodeWorkerPtr node);

    void emitSelection();

Q_SIGNALS:
    void selectionChanged(QList<NodeWorker*>);

    void nodeAdded(NodeWorkerPtr);
    void nodeRemoved(NodeWorkerPtr);


private:
    std::vector<NodeWorker*> rows_;
};

}

#endif // ACTIVITY_LEGEND_H

