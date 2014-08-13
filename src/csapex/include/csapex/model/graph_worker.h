#ifndef GRAPH_WORKER_H
#define GRAPH_WORKER_H

/// PROJECT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QObject>

namespace csapex
{

class GraphWorker : public QObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<GraphWorker> Ptr;

public:
    GraphWorker(GraphPtr graph);

private:
    GraphPtr graph_;
};

}

#endif // GRAPH_WORKER_H
