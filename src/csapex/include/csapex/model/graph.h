#ifndef GRAPH_H
#define GRAPH_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <boost/signals2/signal.hpp>
#include <map>
#include <functional>

namespace csapex {

class Graph
{
    friend class GraphIO;
    friend class GraphWorker;

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

    std::vector<NodeWorker*> getAllNodeWorkers();

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

    // TODO: do this correctly.. -> iterators
    void foreachNode(std::function<void (NodeWorker*)> f);
    void foreachNode(std::function<void (NodeWorker*)> f, std::function<bool (NodeWorker*)> pred);

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
