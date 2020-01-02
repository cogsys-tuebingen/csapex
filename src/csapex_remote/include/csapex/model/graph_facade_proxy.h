#ifndef GRAPH_FACADE_PROXY_H
#define GRAPH_FACADE_PROXY_H

#include <csapex/model/graph_facade.h>
#include <csapex/io/io_fwd.h>
#include <csapex/io/proxy.h>

namespace csapex
{
class GraphFacadeImplementation;
class GraphProxy;

class GraphFacadeProxy : public GraphFacade, public Proxy
{
public:
    GraphFacadeProxy(const SessionPtr& session, NodeFacadeProxyPtr remote_facade, GraphFacadeProxy* parent = nullptr);
    ~GraphFacadeProxy() override;

    AUUID getAbsoluteUUID() const override;

    UUID generateUUID(const std::string& prefix) override;

    NodeFacadePtr getNodeFacade() const override;

    GraphFacadePtr getSubGraph(const UUID& uuid) override;
    GraphFacade* getParent() const override;

    NodeFacadePtr findNodeFacade(const UUID& uuid) const override;
    NodeFacadePtr findNodeFacadeNoThrow(const UUID& uuid) const noexcept override;
    NodeFacadePtr findNodeFacadeForConnector(const UUID& uuid) const override;
    NodeFacadePtr findNodeFacadeForConnectorNoThrow(const UUID& uuid) const noexcept override;
    NodeFacadePtr findNodeFacadeWithLabel(const std::string& label) const override;

    ConnectorPtr findConnector(const UUID& uuid) override;
    ConnectorPtr findConnectorNoThrow(const UUID& uuid) noexcept override;

    bool isConnected(const UUID& from, const UUID& to) const override;
    ConnectionDescription getConnection(const UUID& from, const UUID& to) const override;
    ConnectionDescription getConnectionWithId(int id) const override;

    std::size_t countNodes() const override;

    int getComponent(const UUID& node_uuid) const override;
    int getDepth(const UUID& node_uuid) const override;

    GraphFacadeProxy* getProxyParent() const;

    std::vector<UUID> enumerateAllNodes() const override;
    std::vector<ConnectionDescription> enumerateAllConnections() const override;

    void clearBlock() override;
    void resetActivity() override;

    void pauseRequest(bool pause) override;

/**
 * begin: generate getters
 **/
#define HANDLE_ACCESSOR(_enum, type, function) type function() const override;

#define HANDLE_STATIC_ACCESSOR(_enum, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_SIGNAL(_enum, signal)

#include <csapex/model/graph_facade_proxy_accessors.hpp>
    /**
     * end: generate getters
     **/

protected:
    void nodeAddedHandler(graph::VertexPtr node) override;
    void nodeRemovedHandler(graph::VertexPtr node) override;

    void handleBroadcast(const BroadcastMessageConstPtr& message) override;

    void createInternalConnector(const ConnectorDescription& cd);
    void removeInternalConnector(const ConnectorDescription& cd);

    void createSubgraphFacade(NodeFacadePtr nf);
    void destroySubgraphFacade(NodeFacadePtr nf);

private:
    GraphFacadeProxy* parent_;
    io::ChannelPtr graph_channel_;

    NodeFacadeProxyPtr graph_handle_;

    std::shared_ptr<GraphProxy> graph_;

    AUUID uuid_;

/**
 * begin: generate caches
 **/
#define HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_STATIC_ACCESSOR(_enum, type, function)                                                                                                                                                  \
    mutable bool has_##function##_;                                                                                                                                                                    \
    mutable type cache_##function##_;
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function)                                                                                                                                         \
    mutable bool has_##function##_;                                                                                                                                                                    \
    mutable type value_##function##_;
#define HANDLE_SIGNAL(_enum, signal)

#include <csapex/model/graph_facade_proxy_accessors.hpp>
    /**
     * end: generate caches
     **/

    std::unordered_map<UUID, std::shared_ptr<GraphFacadeProxy>, UUID::Hasher> children_;

    long guard_;
};

}  // namespace csapex

#endif  // GRAPH_FACADE_PROXY_H
