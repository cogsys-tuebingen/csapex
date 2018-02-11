#ifndef GRAPH_WORKER_H
#define GRAPH_WORKER_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex_core/csapex_core_export.h>
#include <csapex/model/notifier.h>
#include <csapex/model/notification.h>
#include <csapex/utility/slim_signal.hpp>
#include <csapex/model/observer.h>
#include <csapex/model/connection_description.h>

/// SYSTEM
#include <unordered_map>

namespace csapex
{

class CSAPEX_CORE_EXPORT GraphFacade : public Observer, public Notifier
{
public:
    typedef std::shared_ptr<GraphFacade> Ptr;

protected:
    GraphFacade();

public:
    virtual ~GraphFacade();

    virtual GraphFacadePtr getSubGraph(const UUID& uuid) = 0;
    virtual GraphFacade* getParent() const = 0;    

    virtual NodeFacadePtr findNodeFacade(const UUID& uuid) const = 0;
    virtual NodeFacadePtr findNodeFacadeNoThrow(const UUID& uuid) const noexcept = 0;
    virtual NodeFacadePtr findNodeFacadeForConnector(const UUID &uuid) const = 0;
    virtual NodeFacadePtr findNodeFacadeForConnectorNoThrow(const UUID &uuid) const noexcept = 0;
    virtual NodeFacadePtr findNodeFacadeWithLabel(const std::string& label) const = 0;

    virtual ConnectorPtr findConnector(const UUID &uuid) = 0;
    virtual ConnectorPtr findConnectorNoThrow(const UUID &uuid) noexcept = 0;

    virtual bool isConnected(const UUID& from, const UUID& to) const = 0;
    virtual ConnectionDescription getConnection(const UUID& from, const UUID& to) const = 0;
    virtual ConnectionDescription getConnectionWithId(int id) const = 0;

    virtual std::vector<UUID> enumerateAllNodes() const = 0;
    virtual std::vector<ConnectionDescription> enumerateAllConnections() const = 0;

    virtual AUUID getAbsoluteUUID() const = 0;

    virtual UUID generateUUID(const std::string& prefix) = 0;

    virtual std::size_t countNodes() const = 0;

    virtual int getComponent(const UUID& node_uuid) const = 0;
    virtual int getDepth(const UUID& node_uuid) const = 0;

    virtual NodeFacadePtr getNodeFacade() const = 0;

    virtual void clearBlock() = 0;
    virtual void resetActivity() = 0;

    virtual bool isPaused() const = 0;
    virtual void pauseRequest(bool pause) = 0;

    virtual std::string makeStatusString() const = 0;

public:
    slim_signal::Signal<void (bool)> paused;
    slim_signal::Signal<void ()> stopped;

    slim_signal::Signal<void(GraphFacadePtr)> child_added;
    slim_signal::Signal<void(GraphFacadePtr)> child_removed;

    slim_signal::Signal<void(NodeFacadePtr)> node_facade_added;
    slim_signal::Signal<void(NodeFacadePtr)> node_facade_removed;

    slim_signal::Signal<void(NodeFacadePtr)> child_node_facade_added;
    slim_signal::Signal<void(NodeFacadePtr)> child_node_facade_removed;

    slim_signal::Signal<void()> state_changed;

    slim_signal::Signal<void(ConnectionDescription)> connection_added;
    slim_signal::Signal<void(ConnectionDescription)> connection_removed;

    slim_signal::Signal<void(ConnectorPtr)> forwarding_connector_added;
    slim_signal::Signal<void(ConnectorPtr)> forwarding_connector_removed;
    
protected:
    virtual void nodeAddedHandler(graph::VertexPtr node) = 0;
    virtual void nodeRemovedHandler(graph::VertexPtr node) = 0;
};

}

#endif // GRAPH_WORKER_H
