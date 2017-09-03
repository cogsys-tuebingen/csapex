/// HEADER
#include <csapex/model/graph_facade_remote.h>

/// PROJECT
#include <csapex/model/graph_facade_local.h>
#include <csapex/model/graph/graph_remote.h>
#include <csapex/model/graph/graph_local.h>
#include <csapex/io/session.h>

using namespace csapex;

GraphFacadeRemote::GraphFacadeRemote(SessionPtr session, GraphFacadeLocal& tmp_ref, GraphFacadeRemote* parent)
    : GraphFacade(tmp_ref.getNodeFacade()),
      session_(session),
      parent_(parent),
      graph_(std::make_shared<GraphRemote>(session, tmp_ref.getAbsoluteUUID(),
                                           *std::dynamic_pointer_cast<GraphLocal>(tmp_ref.getGraph()))),
      tmp_ref_(tmp_ref)
{
    observe(graph_->vertex_added, delegate::Delegate<void(graph::VertexPtr)>(this, &GraphFacadeRemote::nodeAddedHandler));
    observe(graph_->vertex_removed, delegate::Delegate<void(graph::VertexPtr)>(this, &GraphFacadeRemote::nodeRemovedHandler));

    observe(graph_->notification, notification);

    observe(tmp_ref.paused, paused);
    observe(tmp_ref.stopped, stopped);

    observe(tmp_ref.panic, panic);

    observe(tmp_ref.forwardingAdded, forwardingAdded);
    observe(tmp_ref.forwardingRemoved, forwardingRemoved);
    observe(tmp_ref.internalConnectionInProgress, internalConnectionInProgress);

    //TODO: these have to be translated

    observe(tmp_ref.child_added, child_added);
    observe(tmp_ref.child_removed, child_removed);

    observe(tmp_ref.node_facade_added, node_facade_added);
    observe(tmp_ref.node_facade_removed, node_facade_removed);

    observe(tmp_ref.child_node_facade_added, child_node_facade_added);
    observe(tmp_ref.child_node_facade_removed, child_node_facade_removed);
}


AUUID GraphFacadeRemote::getAbsoluteUUID() const
{
    return tmp_ref_.getAbsoluteUUID();
}

GraphFacade* GraphFacadeRemote::getParent() const
{
    return parent_;
}

GraphFacade* GraphFacadeRemote::getSubGraph(const UUID &uuid)
{
    throw std::runtime_error("not implemented yet...");
}

GraphPtr GraphFacadeRemote::getGraph() const
{
    return graph_;
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

void GraphFacadeRemote::stop()
{
    // TODO: implement client server
    tmp_ref_.stop();
}
void GraphFacadeRemote::clear()
{
    // TODO: implement client server
    tmp_ref_.clear();
}

void GraphFacadeRemote::clearBlock()
{
    // TODO: implement client server
    tmp_ref_.clearBlock();
}

void GraphFacadeRemote::resetActivity()
{
    // TODO: implement client server
    tmp_ref_.resetActivity();
}

bool GraphFacadeRemote::isPaused() const
{
    // TODO: implement client server
    return tmp_ref_.isPaused();
}
void GraphFacadeRemote::pauseRequest(bool pause)
{
    // TODO: implement client server
    tmp_ref_.pauseRequest(pause);
}


std::string GraphFacadeRemote::makeStatusString()
{
    return tmp_ref_.makeStatusString();
}
