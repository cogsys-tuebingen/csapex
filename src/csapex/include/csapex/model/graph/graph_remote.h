#ifndef GRAPH_REMOTE_H
#define GRAPH_REMOTE_H

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/observer.h>
#include <csapex/io/io_fwd.h>
#include <csapex/io/remote.h>

namespace csapex
{

class GraphLocal;

class GraphRemote : public Graph, public Observer
{
public:
    GraphRemote(io::ChannelPtr channel, const AUUID &auuid, NodeFacadeRemotePtr &node_facade);
    ~GraphRemote();

    AUUID getAbsoluteUUID() const override;

    int getComponent(const UUID& node_uuid) const override;
    int getDepth(const UUID& node_uuid) const override;

    NodeFacadePtr findNodeFacade(const UUID& uuid) const override;
    NodeFacadePtr findNodeFacadeNoThrow(const UUID& uuid) const noexcept override;
    NodeFacadePtr findNodeFacadeForConnector(const UUID &uuid) const override;
    NodeFacadePtr findNodeFacadeForConnectorNoThrow(const UUID &uuid) const noexcept override;
    NodeFacadePtr findNodeFacadeWithLabel(const std::string& label) const override;

    std::vector<UUID> getAllNodeUUIDs() const override;
    std::vector<NodeFacadePtr> getAllNodeFacades() override;


    ConnectorPtr findConnector(const UUID &uuid) override;
    ConnectorPtr findConnectorNoThrow(const UUID &uuid) noexcept override;

    bool isConnected(const UUID& from, const UUID& to) const override;

    virtual std::vector<ConnectionInformation> enumerateAllConnections() const override;

    int countNodes() override;

    ConnectionInformation getConnection(const UUID& from, const UUID& to) const;
    ConnectionInformation getConnectionWithId(int id) const;

    void reload();

    /**
     * begin: generate getters
     **/
    #define HANDLE_ACCESSOR(_enum, type, function) \
    virtual type function() const override;

    #define HANDLE_STATIC_ACCESSOR(_enum, type, function) HANDLE_ACCESSOR(_enum, type, function)
    #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) HANDLE_ACCESSOR(_enum, type, function)
    #define HANDLE_SIGNAL(_enum, signal)

    #include <csapex/model/graph_remote_accessors.hpp>
    /**
     * end: generate getters
     **/

private:
    void vertexAdded(const UUID &id);
    void vertexRemoved(const UUID &id);

    void connectionAdded(const ConnectionInformation& id);
    void connectionRemoved(const ConnectionInformation& id);

private:
    io::ChannelPtr graph_channel_;

    std::vector<graph::VertexPtr> remote_vertices_;
    std::vector<ConnectionInformation> edges_;

    /**
     * begin: generate caches
     **/
    #define HANDLE_ACCESSOR(_enum, type, function)
    #define HANDLE_STATIC_ACCESSOR(_enum, type, function) \
    mutable bool has_##function##_; \
    mutable type cache_##function##_;
    #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
    mutable bool has_##function##_; \
    mutable type value_##function##_;
    #define HANDLE_SIGNAL(_enum, signal)

    #include <csapex/model/graph_remote_accessors.hpp>
    /**
     * end: generate caches
     **/

    NodeFacadeRemotePtr nf_;
};

}

#endif // GRAPH_REMOTE_H
