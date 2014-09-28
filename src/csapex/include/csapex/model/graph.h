#ifndef GRAPH_H
#define GRAPH_H

/// COMPONENT
#include <csapex/model/connection.h>
#include <csapex/command/command.h>
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <QObject>
#include <map>
#include <boost/function.hpp>

namespace csapex {

class Graph : public QObject
{
    Q_OBJECT

    friend class GraphIO;
    friend class GraphWorker;
    friend class command::AddNode;
    friend class command::AddConnection;
    friend class command::DeleteConnection;
    friend class command::DeleteNode;

    /*remove*/ friend class DesignerScene;
    /*remove*/ friend class WidgetController;

public:
    typedef boost::shared_ptr<Graph> Ptr;

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

    Node* findNode(const UUID& uuid) const;
    Node* findNodeNoThrow(const UUID& uuid) const;
    Node* findNodeForConnector(const UUID &uuid) const;

    NodeWorker* findNodeWorker(const UUID& uuid) const;
    NodeWorker* findNodeWorkerNoThrow(const UUID& uuid) const;
    NodeWorker* findNodeWorkerForConnector(const UUID &uuid) const;

    std::vector<NodeWorker*> getAllNodeWorkers();

    int getComponent(const UUID& node_uuid) const;

    Connectable* findConnector(const UUID &uuid);

    // TODO: extract commands from here!!
    Command::Ptr deleteConnectionByIdCommand(int id);
    Command::Ptr deleteConnectionFulcrumCommand(int connection, int fulcrum);
    Command::Ptr deleteAllConnectionFulcrumsCommand(int connection);
    Command::Ptr deleteAllConnectionFulcrumsCommand(Connection::Ptr connection);
    Command::Ptr deleteConnectionById(int id);
    Command::Ptr clear();

    Connection::Ptr getConnectionWithId(int id);
    Connection::Ptr getConnection(Connection::Ptr);
    int getConnectionId(Connection::Ptr);

    std::string makeUUIDPrefix(const std::string& name);

    int countNodes();

    void addNode(NodeWorkerPtr node);
    void deleteNode(const UUID &uuid);

    bool addConnection(Connection::Ptr connection);
    void deleteConnection(Connection::Ptr connection);

    // TODO: do this correctly.. -> iterators
    void foreachNode(boost::function<void (Node*)> f);
    void foreachNode(boost::function<void (Node*)> f, boost::function<bool (Node*)> pred);

private:
   /*rename*/ void verify();
    void buildConnectedComponents();

Q_SIGNALS:
    void stateChanged();
    void structureChanged(Graph*);
    /*extract?*/ void dirtyChanged(bool);

    void connectionAdded(Connection*);
    void connectionDeleted(Connection*);

    void nodeAdded(NodeWorkerPtr);
    void nodeRemoved(NodeWorkerPtr);

protected:
    std::vector<NodeWorkerPtr> nodes_;
    std::map<NodeWorker*, int> node_component_;

    std::map<NodeWorker*, std::vector<NodeWorker*> > node_parents_;
    std::map<NodeWorker*, std::vector<NodeWorker*> > node_children_;

    std::vector<Connection::Ptr> connections_;

    std::map<std::string, int> uuids_;
};

}

#endif // GRAPH_H
