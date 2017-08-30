#ifndef GRAPH_WORKER_H
#define GRAPH_WORKER_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/csapex_export.h>
#include <csapex/utility/notifier.h>
#include <csapex/utility/notification.h>
#include <csapex/utility/slim_signal.hpp>
#include <csapex/model/observer.h>

/// SYSTEM
#include <unordered_map>

namespace csapex
{

class CSAPEX_EXPORT GraphFacade : public Observer, public Notifier
{
public:
    typedef std::shared_ptr<GraphFacade> Ptr;

protected:
    GraphFacade(ThreadPool& executor, GraphPtr graph, SubgraphNodePtr graph_node, NodeFacadePtr nh = nullptr);

public:
    virtual ~GraphFacade();

    GraphFacade* getSubGraph(const UUID& uuid);
    virtual GraphFacade* getParent() const = 0;

    AUUID getAbsoluteUUID() const;
    GraphPtr getGraph();
    SubgraphNodePtr getSubgraphNode();
    NodeHandle* getNodeHandle();
    NodeFacadePtr getNodeFacade();
    ThreadPool* getThreadPool();

    void stop();
    virtual void clearBlock() = 0;
    virtual void resetActivity() = 0;

    bool isPaused() const;
    void pauseRequest(bool pause);

    virtual void clear();

public:
    slim_signal::Signal<void (bool)> paused;
    slim_signal::Signal<void ()> stopped;

    slim_signal::Signal<void(GraphFacadePtr)> child_added;
    slim_signal::Signal<void(GraphFacadePtr)> child_removed;

    slim_signal::Signal<void(NodeFacadePtr)> node_facade_added;
    slim_signal::Signal<void(NodeFacadePtr)> node_facade_removed;

    slim_signal::Signal<void(NodeFacadePtr)> child_node_facade_added;
    slim_signal::Signal<void(NodeFacadePtr)> child_node_facade_removed;

    slim_signal::Signal<void()> panic;
    
protected:
    virtual void nodeAddedHandler(graph::VertexPtr node) = 0;
    virtual void nodeRemovedHandler(graph::VertexPtr node) = 0;

    virtual void createSubgraphFacade(NodeFacadePtr nh) = 0;

protected:
    // TODO: move all of this into the implementation class
    AUUID absolute_uuid_;
    GraphPtr graph_;
    SubgraphNodePtr graph_node_;
    NodeFacadePtr graph_handle_;
    ThreadPool& executor_;

    std::unordered_map<UUID, GraphFacadePtr, UUID::Hasher> children_;

    std::unordered_map<UUID, NodeFacadePtr, UUID::Hasher> node_facades_;
};

}

#endif // GRAPH_WORKER_H
