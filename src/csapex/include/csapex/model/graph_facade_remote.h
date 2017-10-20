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
    GraphFacadeRemote(SessionPtr session, AUUID uuid, GraphFacadeLocal& tmp_ref, GraphFacadeRemote *parent = nullptr);

    virtual AUUID getAbsoluteUUID() const override;

    virtual GraphFacade* getSubGraph(const UUID& uuid) override;
    virtual GraphFacade* getParent() const override;

    virtual GraphPtr getGraph() const override;

    GraphFacadeRemote* getRemoteParent() const;

    virtual std::vector<ConnectionInformation> enumerateAllConnections() const override;

    virtual void stop() override;
    virtual void clear() override;
    virtual void clearBlock() override;
    virtual void resetActivity() override;

    virtual bool isPaused() const override;
    virtual void pauseRequest(bool pause) override;

    virtual std::string makeStatusString() override;

protected:
    virtual void nodeAddedHandler(graph::VertexPtr node) override;
    virtual void nodeRemovedHandler(graph::VertexPtr node) override;

    void handleBroadcast(const BroadcastMessageConstPtr& message) override;

private:
    GraphFacadeRemote* parent_;
    std::shared_ptr<GraphRemote> graph_;

    AUUID uuid_;

    io::ChannelPtr graph_channel_;

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

    GraphFacadeLocal& tmp_ref_;
};

}

#endif // GRAPH_FACADE_REMOTE_H
