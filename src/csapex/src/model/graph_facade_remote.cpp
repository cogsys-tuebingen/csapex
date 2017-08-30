/// HEADER
#include <csapex/model/graph_facade_remote.h>

/// PROJECT
#include <csapex/model/graph_facade_local.h>
#include <csapex/model/graph/graph_remote.h>
#include <csapex/model/graph/graph_local.h>
#include <csapex/io/session.h>

using namespace csapex;

GraphFacadeRemote::GraphFacadeRemote(SessionPtr session, GraphFacadeLocal& tmp_ref, GraphFacadeRemote* parent)
    : GraphFacade(*tmp_ref.getThreadPool(),
                  std::make_shared<GraphRemote>(session, *std::dynamic_pointer_cast<GraphLocal>(tmp_ref.getGraph())),
                  tmp_ref.getSubgraphNode(),
                  tmp_ref.getNodeFacade()),
      session_(session),
      parent_(parent),
      tmp_ref_(tmp_ref)
{
    observe(tmp_ref.paused, paused);
    observe(tmp_ref.stopped, stopped);

    observe(tmp_ref.panic, panic);

    //TODO: these have to be translated

    observe(tmp_ref.child_added, child_added);
    observe(tmp_ref.child_removed, child_removed);

    observe(tmp_ref.node_facade_added, node_facade_added);
    observe(tmp_ref.node_facade_removed, node_facade_removed);

    observe(tmp_ref.child_node_facade_added, child_node_facade_added);
    observe(tmp_ref.child_node_facade_removed, child_node_facade_removed);
}

GraphFacade* GraphFacadeRemote::getParent() const
{
    return parent_;
}
GraphFacadeRemote* GraphFacadeRemote::getRemoteParent() const
{
    return parent_;
}

void GraphFacadeRemote::nodeAddedHandler(graph::VertexPtr vertex)
{
    // TODO: implement client server
}

void GraphFacadeRemote::nodeRemovedHandler(graph::VertexPtr vertex)
{
    // TODO: implement client server
}

void GraphFacadeRemote::createSubgraphFacade(NodeFacadePtr nf)
{
    // TODO: implement client server
}

void GraphFacadeRemote::clearBlock()
{
    // TODO: implement client server
}

void GraphFacadeRemote::resetActivity()
{
    // TODO: implement client server
}
