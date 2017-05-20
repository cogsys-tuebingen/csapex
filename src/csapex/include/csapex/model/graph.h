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

    typedef std::vector<graph::VertexPtr>::iterator vertex_iterator;
    typedef std::vector<graph::VertexPtr>::const_iterator vertex_const_iterator;

public:
    virtual ~Graph();

    virtual int getComponent(const UUID& node_uuid) const = 0;
    virtual int getDepth(const UUID& node_uuid) const = 0;

    virtual void resetActivity() = 0;

    virtual void clear() = 0;

    virtual Node* findNode(const UUID& uuid) const = 0;
    virtual Node* findNodeNoThrow(const UUID& uuid) const noexcept = 0;
    virtual Node* findNodeForConnector(const UUID &uuid) const = 0;

    virtual NodeHandle* findNodeHandle(const UUID& uuid) const = 0;
    virtual NodeHandle* findNodeHandleNoThrow(const UUID& uuid) const noexcept = 0;
    virtual NodeHandle* findNodeHandleForConnector(const UUID &uuid) const = 0;
    virtual NodeHandle* findNodeHandleForConnectorNoThrow(const UUID &uuid) const noexcept = 0;
    virtual NodeHandle* findNodeHandleWithLabel(const std::string& label) const = 0;

    virtual std::vector<NodeHandle*> getAllNodeHandles() = 0;


    virtual ConnectablePtr findConnector(const UUID &uuid) = 0;
    virtual ConnectablePtr findConnectorNoThrow(const UUID &uuid) noexcept = 0;

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

    virtual ConnectionPtr getConnectionWithId(int id) = 0;
    virtual ConnectionPtr getConnection(const UUID& from, const UUID& to) = 0;
    virtual ConnectionPtr getConnection(Connectable* from, Connectable* to) = 0;
    virtual int getConnectionId(ConnectionPtr) = 0;

    virtual std::vector<ConnectionPtr> getConnections() = 0;

    virtual int countNodes() = 0;

    virtual void addNode(NodeFacadePtr node) = 0;
    virtual void deleteNode(const UUID &uuid) = 0;

    virtual bool addConnection(ConnectionPtr connection) = 0;
    virtual void deleteConnection(ConnectionPtr connection) = 0;

    virtual void beginTransaction() = 0;
    virtual void finalizeTransaction() = 0;

    virtual void analyzeGraph() = 0;

    // iterators
    virtual vertex_iterator beginVertices() = 0;
    virtual const vertex_const_iterator beginVertices() const = 0;

    virtual vertex_iterator endVertices() = 0;
    virtual const vertex_const_iterator endVertices() const = 0;


protected:
    Graph();

public:
    slim_signal::Signal<void()> state_changed;

    slim_signal::Signal<void(Connection*)> connection_added;
    slim_signal::Signal<void(Connection*)> connection_removed;

    slim_signal::Signal<void(graph::VertexPtr)> vertex_added;
    slim_signal::Signal<void(graph::VertexPtr)> vertex_removed;
};

}

#endif // GRAPH_H
