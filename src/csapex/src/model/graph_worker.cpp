/// HEADER
#include <csapex/model/graph_worker.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/connection.h>
#include <csapex/core/settings.h>

/// SYSTEM
#include <QTimer>

using namespace csapex;

GraphWorker::GraphWorker(Settings* /*settings*/, Graph* graph)
    : graph_(graph), paused_(false)
{
}

Graph* GraphWorker::getGraph()
{
    return graph_;
}

void GraphWorker::reset()
{
    stop();

    graph_->uuids_.clear();
    graph_->connections_.clear();
}

bool GraphWorker::isPaused() const
{
    return paused_;
}

void GraphWorker::setPause(bool pause)
{
    if(pause == isPaused()) {
        return;
    }

    paused_ = pause;

    for(NodeWorker* node : graph_->getAllNodeWorkers()) {
        node->pause(pause);
    }

    paused(pause);
}


void GraphWorker::stop()
{
//    for(NodeWorker* node : graph_->getAllNodeWorkers()) {
//        node->setEnabled(false);
//    }
    for(NodeWorker* node : graph_->getAllNodeWorkers()) {
        node->stop();
    }

    graph_->nodes_.clear();
}
