#ifndef GRAPH_FACADE_REMOTE_H
#define GRAPH_FACADE_REMOTE_H

#include <csapex/model/graph_facade.h>
#include <csapex/io/io_fwd.h>
#include <csapex/io/remote.h>

namespace csapex
{
class GraphFacadeLocal;
class GraphRemote;

class GraphFacadeRemote : public GraphFacade, public Remote
{
public:
    GraphFacadeRemote(const SessionPtr &session, NodeFacadeRemotePtr remote_facade, GraphFacadeRemote *parent = nullptr);
    ~GraphFacadeRemote();

    virtual AUUID getAbsoluteUUID() const override;

    virtual UUID generateUUID(const std::string& prefix) override;

    virtual NodeFacadePtr getNodeFacade() const override;

    virtual GraphFacade* getSubGraph(const UUID& uuid) override;
    virtual GraphFacade* getParent() const override;

    virtual NodeFacadePtr findNodeFacade(const UUID& uuid) const override;
    virtual NodeFacadePtr findNodeFacadeNoThrow(const UUID& uuid) const noexcept override;
    virtual NodeFacadePtr findNodeFacadeForConnector(const UUID &uuid) const override;
    virtual NodeFacadePtr findNodeFacadeForConnectorNoThrow(const UUID &uuid) const noexcept override;
    virtual NodeFacadePtr findNodeFacadeWithLabel(const std::string& label) const override;

    virtual ConnectorPtr findConnector(const UUID &uuid) override;
    virtual ConnectorPtr findConnectorNoThrow(const UUID &uuid) noexcept override;

    virtual bool isConnected(const UUID& from, const UUID& to) const override;
    virtual ConnectionInformation getConnection(const UUID& from, const UUID& to) const override;
    virtual ConnectionInformation getConnectionWithId(int id) const override;

    virtual std::size_t countNodes() const override;

    virtual int getComponent(const UUID& node_uuid) const override;
    virtual int getDepth(const UUID& node_uuid) const override;

    GraphFacadeRemote* getRemoteParent() const;

    virtual std::vector<UUID> enumerateAllNodes() const override;
    virtual std::vector<ConnectionInformation> enumerateAllConnections() const override;

    virtual void clearBlock() override;
    virtual void resetActivity() override;

    virtual void pauseRequest(bool pause) override;


    /**
     * begin: generate getters
     **/
    #define HANDLE_ACCESSOR(_enum, type, function) \
    virtual type function() const override;

    #define HANDLE_STATIC_ACCESSOR(_enum, type, function) HANDLE_ACCESSOR(_enum, type, function)
    #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) HANDLE_ACCESSOR(_enum, type, function)
    #define HANDLE_SIGNAL(_enum, signal)

    #include <csapex/model/graph_facade_remote_accessors.hpp>
    /**
     * end: generate getters
     **/

protected:
    virtual void nodeAddedHandler(graph::VertexPtr node) override;
    virtual void nodeRemovedHandler(graph::VertexPtr node) override;

    void handleBroadcast(const BroadcastMessageConstPtr& message) override;

    void createInternalConnector(const ConnectorDescription& cd);
    void removeInternalConnector(const ConnectorDescription& cd);

    void createSubgraphFacade(NodeFacadePtr nf);
    void destroySubgraphFacade(NodeFacadePtr nf);

private:
    GraphFacadeRemote* parent_;
    io::ChannelPtr graph_channel_;

    NodeFacadeRemotePtr graph_handle_;

    std::shared_ptr<GraphRemote> graph_;

    AUUID uuid_;

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

    #include <csapex/model/graph_facade_remote_accessors.hpp>
    /**
     * end: generate caches
     **/

    std::unordered_map<UUID, std::shared_ptr<GraphFacadeRemote>, UUID::Hasher> children_;

    long guard_;
};

}

#endif // GRAPH_FACADE_REMOTE_H
