#ifndef GRAPH_PROXY_H
#define GRAPH_PROXY_H

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/observer.h>
#include <csapex/io/io_fwd.h>
#include <csapex/io/proxy.h>

namespace csapex
{
class GraphImplementation;

class GraphProxy : public Graph, public Observer
{
public:
    GraphProxy(io::ChannelPtr channel, NodeFacadeProxyPtr& node_facade);
    ~GraphProxy() override;

    AUUID getAbsoluteUUID() const override;

    int getComponent(const UUID& node_uuid) const override;
    int getDepth(const UUID& node_uuid) const override;

    NodeFacadePtr findNodeFacade(const UUID& uuid) const override;
    NodeFacadePtr findNodeFacadeNoThrow(const UUID& uuid) const noexcept override;
    NodeFacadePtr findNodeFacadeForConnector(const UUID& uuid) const override;
    NodeFacadePtr findNodeFacadeForConnectorNoThrow(const UUID& uuid) const noexcept override;
    NodeFacadePtr findNodeFacadeWithLabel(const std::string& label) const override;

    std::vector<UUID> getAllNodeUUIDs() const override;
    std::vector<NodeFacadePtr> getAllNodeFacades() override;

    ConnectorPtr findConnector(const UUID& uuid) override;
    ConnectorPtr findConnectorNoThrow(const UUID& uuid) noexcept override;

    bool isConnected(const UUID& from, const UUID& to) const override;

    std::vector<ConnectionDescription> enumerateAllConnections() const override;

    std::size_t countNodes() override;

    ConnectionDescription getConnection(const UUID& from, const UUID& to) const;
    ConnectionDescription getConnectionWithId(int id) const;

    void reload();

/**
 * begin: generate getters
 **/
#define HANDLE_ACCESSOR(_enum, type, function) type function() const override;

#define HANDLE_STATIC_ACCESSOR(_enum, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_SIGNAL(_enum, signal)

#include <csapex/model/graph_proxy_accessors.hpp>
    /**
     * end: generate getters
     **/

private:
    void vertexAdded(const UUID& id);
    void vertexRemoved(const UUID& id);

    void connectionAdded(const ConnectionDescription& id);
    void connectionRemoved(const ConnectionDescription& id);

private:
    io::ChannelPtr graph_channel_;

    std::vector<graph::VertexPtr> remote_vertices_;
    std::vector<ConnectionDescription> edges_;

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

#include <csapex/model/graph_proxy_accessors.hpp>
    /**
     * end: generate caches
     **/

    NodeFacadeProxyPtr nf_;
};

}  // namespace csapex

#endif  // GRAPH_PROXY_H
