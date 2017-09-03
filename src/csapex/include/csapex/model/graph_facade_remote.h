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
    GraphFacadeRemote(SessionPtr session, GraphFacadeLocal& tmp_ref, GraphFacadeRemote *parent = nullptr);

    virtual AUUID getAbsoluteUUID() const override;

    virtual GraphFacade* getSubGraph(const UUID& uuid) override;
    virtual GraphFacade* getParent() const override;

    virtual GraphPtr getGraph() const override;

    GraphFacadeRemote* getRemoteParent() const;

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

    std::unordered_map<UUID, std::shared_ptr<GraphFacadeRemote>, UUID::Hasher> children_;

    GraphFacadeLocal& tmp_ref_;
};

}

#endif // GRAPH_FACADE_REMOTE_H
