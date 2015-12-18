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

    struct NodeHandleNotFoundException : public std::logic_error
    {
        NodeHandleNotFoundException(const std::string& name)
            : std::logic_error("node handle for node " + name + " cannot be found")
        {}
    };

    typedef std::vector<NodeHandlePtr>::iterator node_iterator;
    typedef std::vector<NodeHandlePtr>::const_iterator node_const_iterator;

public:
    Graph();
    virtual ~Graph();

    void clear();

    Node* findNode(const UUID& uuid) const;
    Node* findNodeNoThrow(const UUID& uuid) const;
    Node* findNodeForConnector(const UUID &uuid) const;

    NodeHandle* findNodeHandle(const UUID& uuid) const;
    NodeHandle* findNodeHandleNoThrow(const UUID& uuid) const;
    NodeHandle* findNodeHandleForConnector(const UUID &uuid) const;

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

    void addNode(NodeHandlePtr node);
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

    boost::signals2::signal<void(Connection*)> connectionAdded;
    boost::signals2::signal<void(Connection*)> connectionDeleted;

    boost::signals2::signal<void(NodeHandlePtr)> nodeAdded;
    boost::signals2::signal<void(NodeHandlePtr)> nodeRemoved;

protected:
    std::vector<NodeHandlePtr> nodes_;
    std::map<NodeHandle*, int> node_component_;
    std::map<NodeHandle*, int> node_level_;

    std::map<NodeHandle*, std::vector<NodeHandle*> > node_parents_;
    std::map<NodeHandle*, std::vector<NodeHandle*> > node_children_;

    std::vector<ConnectionPtr> connections_;

    std::map<std::string, int> uuids_;
};

}

#endif // GRAPH_H
