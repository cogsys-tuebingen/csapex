#ifndef GRAPH_H
#define GRAPH_H

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <QObject>
#include <map>
#include <functional>

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

    Node* findNode(const UUID& uuid) const;
    Node* findNodeNoThrow(const UUID& uuid) const;
    Node* findNodeForConnector(const UUID &uuid) const;

    NodeWorker* findNodeWorker(const UUID& uuid) const;
    NodeWorker* findNodeWorkerNoThrow(const UUID& uuid) const;
    NodeWorker* findNodeWorkerForConnector(const UUID &uuid) const;

    std::vector<NodeWorker*> getAllNodeWorkers();

    int getComponent(const UUID& node_uuid) const;
    int getLevel(const UUID& node_uuid) const;

    Connectable* findConnector(const UUID &uuid);

    // TODO: extract commands from here!!
    Command::Ptr deleteConnectionByIdCommand(int id);
    Command::Ptr deleteConnectionFulcrumCommand(int connection, int fulcrum);
    Command::Ptr deleteAllConnectionFulcrumsCommand(int connection);
    Command::Ptr deleteAllConnectionFulcrumsCommand(ConnectionPtr connection);
    Command::Ptr deleteConnectionById(int id);
    Command::Ptr clear();

    ConnectionPtr getConnectionWithId(int id);
    ConnectionPtr getConnection(Connectable* from, Connectable* to);
    int getConnectionId(ConnectionPtr);

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

Q_SIGNALS:
    void stateChanged();
    void structureChanged(Graph*);

    void panic();

    void connectionAdded(Connection*);
    void connectionDeleted(Connection*);

    void nodeAdded(NodeWorkerPtr);
    void nodeRemoved(NodeWorkerPtr);

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
