#ifndef GRAPH_FACADE_REMOTE_H
#define GRAPH_FACADE_REMOTE_H

#include <csapex/model/graph_facade.h>
#include <csapex/io/io_fwd.h>

namespace csapex
{
class GraphFacadeLocal;

class GraphFacadeRemote : public GraphFacade
{
public:
    GraphFacadeRemote(SessionPtr session, GraphFacadeLocal& tmp_ref, GraphFacadeRemote *parent = nullptr);

    virtual GraphFacade* getParent() const override;
    GraphFacadeRemote* getRemoteParent() const;

    virtual void clearBlock() override;
    virtual void resetActivity() override;

protected:
    virtual void nodeAddedHandler(graph::VertexPtr node) override;
    virtual void nodeRemovedHandler(graph::VertexPtr node) override;

    virtual void createSubgraphFacade(NodeFacadePtr nf) override;

private:
    SessionPtr session_;

    GraphFacadeRemote* parent_;
    GraphFacadeLocal& tmp_ref_;
};

}

#endif // GRAPH_FACADE_REMOTE_H
