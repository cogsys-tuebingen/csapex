#ifndef GRAPH_SERVER_H
#define GRAPH_SERVER_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/io/remote_io_fwd.h>
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

    void startObservingGraph(const GraphFacadeImplementationPtr& graph);
    void stopObservingGraph(const GraphFacadeImplementationPtr& graph);

private:
    SessionPtr session_;
    NodeServerPtr node_server_;
};
}  // namespace csapex

#endif  // GRAPH_SERVER_H
