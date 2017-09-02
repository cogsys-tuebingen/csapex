#ifndef GRAPH_REMOTE_H
#define GRAPH_REMOTE_H

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/observer.h>
#include <csapex/io/io_fwd.h>

namespace csapex
{

class GraphLocal;

class GraphRemote : public Graph, public Observer
{
public:
    GraphRemote(SessionPtr session, GraphLocal& temp_reference);
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

    NodeFacadePtr findNodeFacade(const UUID& uuid) const override;
    NodeFacadePtr findNodeFacadeNoThrow(const UUID& uuid) const noexcept override;
    NodeFacadePtr findNodeFacadeForConnector(const UUID &uuid) const override;
    NodeFacadePtr findNodeFacadeForConnectorNoThrow(const UUID &uuid) const noexcept override;
    NodeFacadePtr findNodeFacadeWithLabel(const std::string& label) const override;

    Graph* findSubgraph(const UUID& uuid) const override;

    std::vector<NodeHandle*> getAllNodeHandles() override;
    std::vector<NodeFacadePtr> getAllNodeFacades() override;


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
    vertex_iterator begin() override;
    const vertex_const_iterator begin() const override;

    vertex_iterator end() override;
    const vertex_const_iterator end() const override;

private:
    SessionPtr session_;
    GraphLocal& temp_reference;

    std::vector<graph::VertexPtr> remote_vertices_;
};

}

#endif // GRAPH_REMOTE_H
