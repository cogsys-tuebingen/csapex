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
    GraphFacade(NodeFacadePtr nh = nullptr);

public:
    virtual ~GraphFacade();

    virtual GraphFacade* getSubGraph(const UUID& uuid) = 0;
    virtual GraphFacade* getParent() const = 0;    

    NodeFacadePtr findNodeFacade(const UUID& uuid) const;
    NodeFacadePtr findNodeFacadeNoThrow(const UUID& uuid) const noexcept;
    NodeFacadePtr findNodeFacadeForConnector(const UUID &uuid) const;
    NodeFacadePtr findNodeFacadeForConnectorNoThrow(const UUID &uuid) const noexcept;
    NodeFacadePtr findNodeFacadeWithLabel(const std::string& label) const;

    ConnectorPtr findConnector(const UUID &uuid);
    ConnectorPtr findConnectorNoThrow(const UUID &uuid) noexcept;

    virtual AUUID getAbsoluteUUID() const = 0;
    virtual GraphPtr getGraph() const = 0;
    NodeHandle* getNodeHandle();
    NodeFacadePtr getNodeFacade();

    virtual void stop() = 0;
    virtual void clearBlock() = 0;
    virtual void resetActivity() = 0;

    virtual bool isPaused() const = 0;
    virtual void pauseRequest(bool pause) = 0;

    virtual void clear() = 0;

    virtual std::string makeStatusString() = 0;

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

    slim_signal::Signal<void(ConnectorPtr)> forwardingAdded;
    slim_signal::Signal<void(ConnectorPtr)> forwardingRemoved;

    slim_signal::Signal<void(ConnectorPtr,ConnectorPtr)> internalConnectionInProgress;
    
protected:
    virtual void nodeAddedHandler(graph::VertexPtr node) = 0;
    virtual void nodeRemovedHandler(graph::VertexPtr node) = 0;

    virtual void createSubgraphFacade(NodeFacadePtr nh) = 0;

protected:
    // TODO: move all of this into the implementation class
    NodeFacadePtr graph_handle_;
};

}

#endif // GRAPH_WORKER_H
