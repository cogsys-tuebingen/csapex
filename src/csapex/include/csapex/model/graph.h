#ifndef GRAPH_H
#define GRAPH_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/csapex_export.h>
#include <csapex/utility/notifier.h>

/// SYSTEM
#include <csapex/utility/slim_signal.hpp>
#include <map>
#include <functional>
#include <set>

namespace csapex {

class CSAPEX_EXPORT Graph : public UUIDProvider, public Notifier
{
    friend class GraphIO;
    friend class GraphFacade;

public:
    typedef std::shared_ptr<Graph> Ptr;

    struct NodeNotFoundException : public std::logic_error
    {
        NodeNotFoundException(const std::string& name)
            : std::logic_error("node " + name + " cannot be found")
        {}
    };

    struct NodeHandleNotFoundException : public std::logic_error
    {
        NodeHandleNotFoundException(const std::string& name)
            : std::logic_error("node handle for node " + name + " cannot be found")
        {}
    };

    struct NodeFacadeNotFoundException : public std::logic_error
    {
        NodeFacadeNotFoundException(const std::string& name)
            : std::logic_error("node facade for node " + name + " cannot be found")
        {}
    };

    typedef std::vector<graph::VertexPtr>::iterator vertex_iterator;
    typedef std::vector<graph::VertexPtr>::const_iterator vertex_const_iterator;

public:
    virtual ~Graph();

    virtual AUUID getAbsoluteUUID() const = 0;

    virtual int getComponent(const UUID& node_uuid) const = 0;
    virtual int getDepth(const UUID& node_uuid) const = 0;

    virtual NodeFacadePtr findNodeFacade(const UUID& uuid) const = 0;
    virtual NodeFacadePtr findNodeFacadeNoThrow(const UUID& uuid) const noexcept = 0;
    virtual NodeFacadePtr findNodeFacadeForConnector(const UUID &uuid) const = 0;
    virtual NodeFacadePtr findNodeFacadeForConnectorNoThrow(const UUID &uuid) const noexcept = 0;
    virtual NodeFacadePtr findNodeFacadeWithLabel(const std::string& label) const = 0;

    virtual ConnectorPtr findConnector(const UUID &uuid) = 0;
    virtual ConnectorPtr findConnectorNoThrow(const UUID &uuid) noexcept = 0;

    virtual std::vector<UUID> getAllNodeUUIDs() const = 0;
    virtual std::vector<NodeFacadePtr> getAllNodeFacades() = 0;


    template <typename T>
    std::shared_ptr<T> findTypedConnector(const UUID &uuid)
    {
        return std::dynamic_pointer_cast<T>(findConnector(uuid));
    }
    template <typename T>
    std::shared_ptr<T> findTypedConnectorNoThrow(const UUID &uuid) noexcept
    {
        return std::dynamic_pointer_cast<T>(findConnectorNoThrow(uuid));
    }

    virtual bool isConnected(const UUID& from, const UUID& to) const = 0;

    virtual int countNodes() = 0;

    virtual std::vector<ConnectionInformation> enumerateAllConnections() const = 0;

protected:
    Graph();

public:
    slim_signal::Signal<void()> state_changed;

    slim_signal::Signal<void(ConnectionInformation)> connection_added;
    slim_signal::Signal<void(ConnectionInformation)> connection_removed;

    slim_signal::Signal<void(graph::VertexPtr)> vertex_added;
    slim_signal::Signal<void(graph::VertexPtr)> vertex_removed;
};

}

#endif // GRAPH_H
