#ifndef GRAPH_H
#define GRAPH_H

/// COMPONENT
#include <csapex/utility/uuid.h>
#include <csapex/model/model_fwd.h>

/// SYSTEM
#include <boost/signals2/signal.hpp>
#include <map>
#include <functional>

namespace csapex {

class Graph
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

    struct NodeWorkerNotFoundException : public std::logic_error
    {
        NodeWorkerNotFoundException(const std::string& name)
            : std::logic_error("node worker for node " + name + " cannot be found")
        {}
    };

    struct NodeHandleNotFoundException : public std::logic_error
    {
        NodeHandleNotFoundException(const std::string& name)
            : std::logic_error("node handle for node " + name + " cannot be found")
        {}
    };

    typedef std::vector<NodeWorkerPtr>::iterator node_iterator;
    typedef std::vector<NodeWorkerPtr>::const_iterator node_const_iterator;

public:
    Graph();
    virtual ~Graph();

    void clear();

    Node* findNode(const UUID& uuid) const;
    Node* findNodeNoThrow(const UUID& uuid) const;
    Node* findNodeForConnector(const UUID &uuid) const;

    NodeWorker* findNodeWorker(const UUID& uuid) const;
    NodeWorker* findNodeWorkerNoThrow(const UUID& uuid) const;
    NodeWorker* findNodeWorkerForConnector(const UUID &uuid) const;

    NodeHandle* findNodeHandle(const UUID& uuid) const;
    NodeHandle* findNodeHandleNoThrow(const UUID& uuid) const;
    NodeHandle* findNodeHandleForConnector(const UUID &uuid) const;

    std::vector<NodeWorker*> getAllNodeWorkers();
    std::vector<NodeHandle*> getAllNodeHandles();

    int getComponent(const UUID& node_uuid) const;
    int getLevel(const UUID& node_uuid) const;

    Connectable *findConnector(const UUID &uuid);

    ConnectionPtr getConnectionWithId(int id);
    ConnectionPtr getConnection(const UUID& from, const UUID& to);
    ConnectionPtr getConnection(Connectable* from, Connectable* to);
    int getConnectionId(ConnectionPtr);

    std::vector<ConnectionPtr> getConnections();

    std::string makeUUIDPrefix(const std::string& name);

    int countNodes();

    void addNode(NodeWorkerPtr node);
    void deleteNode(const UUID &uuid);

    bool addConnection(ConnectionPtr connection);
    void deleteConnection(ConnectionPtr connection);

    // iterators
    node_iterator beginNodes();
    const node_const_iterator beginNodes() const;

    node_iterator endNodes();
    const node_const_iterator endNodes() const;

private:
   /*rename*/ void verify();
    void buildConnectedComponents();
    void assignLevels();

public:
    boost::signals2::signal<void()> stateChanged;
    boost::signals2::signal<void(Graph*)> structureChanged;

    boost::signals2::signal<void()> panic;

    boost::signals2::signal<void(Connection*)> connectionAdded;
    boost::signals2::signal<void(Connection*)> connectionDeleted;

    boost::signals2::signal<void(NodeWorkerPtr)> nodeAdded;
    boost::signals2::signal<void(NodeWorkerPtr)> nodeRemoved;

protected:
    std::vector<NodeWorkerPtr> nodes_;
    std::map<NodeWorker*, int> node_component_;
    std::map<NodeWorker*, int> node_level_;

    std::map<NodeWorker*, std::vector<NodeWorker*> > node_parents_;
    std::map<NodeWorker*, std::vector<NodeWorker*> > node_children_;

    std::vector<ConnectionPtr> connections_;

    std::map<std::string, int> uuids_;
};

}

#endif // GRAPH_H
