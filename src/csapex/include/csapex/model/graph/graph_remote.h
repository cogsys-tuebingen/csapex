#ifndef GRAPH_REMOTE_H
#define GRAPH_REMOTE_H

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/observer.h>

namespace csapex
{

class GraphLocal;

class GraphRemote : public Graph, public Observer
{
public:
    GraphRemote(GraphLocal& temp_reference);
    ~GraphRemote();

    int getComponent(const UUID& node_uuid) const override;
    int getDepth(const UUID& node_uuid) const override;

    void resetActivity() override;

    void clear() override;

    Node* findNode(const UUID& uuid) const override;
    Node* findNodeNoThrow(const UUID& uuid) const noexcept override;
    Node* findNodeForConnector(const UUID &uuid) const override;

    NodeHandle* findNodeHandle(const UUID& uuid) const override;
    NodeHandle* findNodeHandleNoThrow(const UUID& uuid) const noexcept override;
    NodeHandle* findNodeHandleForConnector(const UUID &uuid) const override;
    NodeHandle* findNodeHandleForConnectorNoThrow(const UUID &uuid) const noexcept override;
    NodeHandle* findNodeHandleWithLabel(const std::string& label) const override;

    std::vector<NodeHandle*> getAllNodeHandles() override;


    ConnectablePtr findConnector(const UUID &uuid) override;
    ConnectablePtr findConnectorNoThrow(const UUID &uuid) noexcept override;

    ConnectionPtr getConnectionWithId(int id) override;
    ConnectionPtr getConnection(const UUID& from, const UUID& to) override;
    ConnectionPtr getConnection(Connectable* from, Connectable* to) override;

    std::vector<ConnectionPtr> getConnections() override;

    int countNodes() override;

    void addNode(NodeFacadePtr node) override;
    void deleteNode(const UUID &uuid) override;

    bool addConnection(ConnectionPtr connection) override;
    void deleteConnection(ConnectionPtr connection) override;


    /*REMOVE*/ void beginTransaction() override;
    /*REMOVE*/ void finalizeTransaction() override;

    /*REMOVE*/ void analyzeGraph() override;

    // iterators
    vertex_iterator beginVertices() override;
    const vertex_const_iterator beginVertices() const override;

    vertex_iterator endVertices() override;
    const vertex_const_iterator endVertices() const override;

private:
    GraphLocal& temp_reference;
};

}

#endif // GRAPH_REMOTE_H
