#ifndef GRAPH_LOCAL_H
#define GRAPH_LOCAL_H

/// COMPONENT
#include <csapex/model/graph.h>

namespace csapex
{

class GraphLocal  : public Graph
{
public:
    GraphLocal();
    ~GraphLocal();

    AUUID getAbsoluteUUID() const override;

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
    std::vector<NodeFacadeLocalPtr> getAllLocalNodeFacades();

    ConnectablePtr findConnectable(const UUID &uuid);

    ConnectorPtr findConnector(const UUID &uuid) override;
    ConnectorPtr findConnectorNoThrow(const UUID &uuid) noexcept override;

    ConnectionPtr getConnectionWithId(int id) override;
    ConnectionPtr getConnection(const UUID& from, const UUID& to) override;

    std::vector<ConnectionPtr> getConnections() override;

    int countNodes() override;

    void addNode(NodeFacadeLocalPtr node);
    void deleteNode(const UUID &uuid);

    bool addConnection(ConnectionPtr connection);
    void deleteConnection(ConnectionPtr connection);

    void beginTransaction();
    void finalizeTransaction();

    void analyzeGraph();

    void setNodeFacade(NodeFacadeLocalWeakPtr nf);

    // iterators
    vertex_iterator begin() override;
    const vertex_const_iterator begin() const override;

    vertex_iterator end() override;
    const vertex_const_iterator end() const override;

private:
    void checkNodeState(NodeHandle* nh);

    void buildConnectedComponents();
    void calculateDepths();

    std::set<graph::Vertex *> findVerticesThatNeedMessages();
    std::set<graph::Vertex *> findVerticesThatJoinStreams();

protected:
//    std::vector<NodeHandlePtr> nodes_;
    std::vector<graph::VertexPtr> vertices_;
    std::vector<ConnectionPtr> edges_;

    std::map<Connection*, std::vector<slim_signal::ScopedConnection>> connection_observations_;

    std::set<graph::VertexPtr> sources_;
    std::set<graph::VertexPtr> sinks_;

    bool in_transaction_;

    NodeFacadeLocalWeakPtr nf_;
};

}

#endif // GRAPH_LOCAL_H
