#ifndef GRAPH_SERVER_H
#define GRAPH_SERVER_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/io/io_fwd.h>
#include <csapex/model/observer.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <unordered_map>

namespace csapex
{
class GraphServer : public Observer
{
public:
    GraphServer(SessionPtr session);
    ~GraphServer();

    void startObserving(const GraphFacadeLocalPtr &graph);
    void stopObserving(const GraphFacadeLocalPtr& graph);

private:
    SessionPtr session_;
    NodeServerPtr node_server_;
};
}

#endif // GRAPH_SERVER_H
