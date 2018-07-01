#ifndef GRAPH_IMPL_H
#define GRAPH_IMPL_H

/// COMPONENT
#include <csapex/model/graph.h>

namespace csapex
{
class GraphImplementation : public Graph
{
public:
    GraphImplementation();
    ~GraphImplementation();

    AUUID getAbsoluteUUID() const override;

    int getComponent(const UUID& node_uuid) const override;
    int getDepth(const UUID& node_uuid) const override;

    void resetActivity();

    void clear();

    Node* findNode(const UUID& uuid) const;
    Node* findNodeNoThrow(const UUID& uuid) const noexcept;
    Node* findNodeForConnector(const UUID& uuid) const;

    NodeHandle* findNodeHandle(const UUID& uuid) const;
    NodeHandle* findNodeHandleNoThrow(const UUID& uuid) const noexcept;
    NodeHandle* findNodeHandleForConnector(const UUID& uuid) const;
    NodeHandle* findNodeHandleForConnectorNoThrow(const UUID& uuid) const noexcept;
    NodeHandle* findNodeHandleWithLabel(const std::string& label) const;

    NodeFacadePtr findNodeFacade(const UUID& uuid) const override;
    NodeFacadePtr findNodeFacadeNoThrow(const UUID& uuid) const noexcept override;
    NodeFacadePtr findNodeFacadeForConnector(const UUID& uuid) const override;
    NodeFacadePtr findNodeFacadeForConnectorNoThrow(const UUID& uuid) const noexcept override;
    NodeFacadePtr findNodeFacadeWithLabel(const std::string& label) const override;

    Graph* findSubgraph(const UUID& uuid) const;

    std::vector<UUID> getAllNodeUUIDs() const override;
    std::vector<NodeFacadePtr> getAllNodeFacades() override;
    std::vector<NodeHandle*> getAllNodeHandles();
    std::vector<NodeFacadeImplementationPtr> getAllLocalNodeFacades();

    ConnectablePtr findConnectable(const UUID& uuid);

    ConnectorPtr findConnector(const UUID& uuid) override;
    ConnectorPtr findConnectorNoThrow(const UUID& uuid) noexcept override;

    bool isConnected(const UUID& from, const UUID& to) const override;
    ConnectionPtr getConnectionWithId(int id);
    ConnectionPtr getConnection(const UUID& from, const UUID& to);

    std::vector<ConnectionPtr> getConnections();
    virtual std::vector<ConnectionDescription> enumerateAllConnections() const override;

    std::size_t countNodes() override;

    void addNode(NodeFacadeImplementationPtr node);
    void deleteNode(const UUID& uuid);

    bool addConnection(ConnectionPtr connection);
    void deleteConnection(ConnectionPtr connection);

    void beginTransaction();
    void finalizeTransaction();

    void analyzeGraph();

    void setNodeFacade(NodeFacadeImplementation* nf);

    // iterators
    vertex_iterator begin();
    const vertex_const_iterator begin() const;

    vertex_iterator end();
    const vertex_const_iterator end() const;

private:
    void checkNodeState(NodeHandle* nh);

    void buildConnectedComponents();
    void calculateDepths();

    std::set<graph::Vertex*> findVerticesThatNeedMessages();
    std::set<graph::Vertex*> findVerticesThatJoinStreams();

protected:
    std::vector<graph::VertexPtr> vertices_;
    std::vector<ConnectionPtr> edges_;

    std::map<Connection*, std::vector<slim_signal::ScopedConnection>> connection_observations_;

    std::set<graph::VertexPtr> sources_;
    std::set<graph::VertexPtr> sinks_;

    bool in_transaction_;

    NodeFacadeImplementation* nf_;
};

}  // namespace csapex

#endif  // GRAPH_IMPL_H
