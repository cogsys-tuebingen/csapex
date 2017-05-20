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

    void beginTransaction() override;
    void finalizeTransaction() override;

    void analyzeGraph() override;

    void setNodeHandle(NodeHandle* nh);

    // iterators
    vertex_iterator beginVertices() override;
    const vertex_const_iterator beginVertices() const override;

    vertex_iterator endVertices() override;
    const vertex_const_iterator endVertices() const override;

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

    NodeHandle* nh_;
};

}

#endif // GRAPH_LOCAL_H
