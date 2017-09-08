/// HEADER
#include <csapex/model/graph_facade_remote.h>

/// PROJECT
#include <csapex/model/graph_facade_local.h>
#include <csapex/model/graph/graph_remote.h>
#include <csapex/model/graph/graph_local.h>
#include <csapex/model/node_facade_remote.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/io/session.h>
#include <csapex/io/protcol/graph_broadcasts.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

GraphFacadeRemote::GraphFacadeRemote(SessionPtr session, GraphFacadeLocal& tmp_ref, GraphFacadeRemote* parent)
    : GraphFacade(tmp_ref.getNodeFacade()),
      Remote(session),
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


    //TODO: these have to be translated

    observe(tmp_ref.child_added, child_added);
    observe(tmp_ref.child_removed, child_removed);

    observe(tmp_ref.node_facade_added, node_facade_added);
    observe(tmp_ref.node_facade_removed, node_facade_removed);

    observe(tmp_ref.child_node_facade_added, child_node_facade_added);
    observe(tmp_ref.child_node_facade_removed, child_node_facade_removed);



//    observe(tmp_ref.child_added, [this](GraphFacadePtr child) {
//        GraphFacadeLocalPtr tmp_child = std::dynamic_pointer_cast<GraphFacadeLocal>(child);
//        apex_assert_hard(tmp_child);

//        UUID uuid = child->getAbsoluteUUID();

//        std::shared_ptr<GraphFacadeRemote> sub_graph_facade = std::make_shared<GraphFacadeRemote>(session_, *tmp_child, this);

//        children_[uuid] = sub_graph_facade;

//        observe(sub_graph_facade->notification, notification);
//        observe(sub_graph_facade->node_facade_added, child_node_facade_added);
//        observe(sub_graph_facade->node_facade_removed, child_node_facade_removed);
//        observe(sub_graph_facade->child_node_facade_added, child_node_facade_added);
//        observe(sub_graph_facade->child_node_facade_removed, child_node_facade_removed);

//        child_added(sub_graph_facade);
//    });
//    observe(tmp_ref.child_removed, [this](GraphFacadePtr child) {
//        GraphFacadeLocalPtr tmp_child = std::dynamic_pointer_cast<GraphFacadeLocal>(child);
//        apex_assert_hard(tmp_child);

//        auto pos = children_.find(tmp_child->getAbsoluteUUID());
//        if(pos != children_.end()) {
//            child_removed(pos->second);
//            children_.erase(pos);
//        }
//    });

//    observe(tmp_ref.node_facade_added, [this](NodeFacadePtr node) {
//        NodeFacadeLocalPtr tmp_node = std::dynamic_pointer_cast<NodeFacadeLocal>(node);
//        apex_assert_hard(tmp_node);

//        NodeFacadePtr remote_node = findNodeFacade(node->getUUID());
//        node_facade_added(remote_node);
//    });
//    observe(tmp_ref.node_facade_removed, [this](NodeFacadePtr node) {
//        NodeFacadeLocalPtr tmp_node = std::dynamic_pointer_cast<NodeFacadeLocal>(node);
//        apex_assert_hard(tmp_node);

//        NodeFacadePtr remote_node = findNodeFacade(node->getUUID());
//        node_facade_removed(remote_node);
//    });
//    observe(tmp_ref.child_node_facade_added, [this](NodeFacadePtr node) {
//        NodeFacadeLocalPtr tmp_node = std::dynamic_pointer_cast<NodeFacadeLocal>(node);
//        apex_assert_hard(tmp_node);

//        NodeFacadePtr remote_node = findNodeFacade(node->getUUID());
//        child_node_facade_added(remote_node);
//    });
//    observe(tmp_ref.child_node_facade_removed, [this](NodeFacadePtr node) {
//        NodeFacadeLocalPtr tmp_node = std::dynamic_pointer_cast<NodeFacadeLocal>(node);
//        apex_assert_hard(tmp_node);

//        NodeFacadePtr remote_node = findNodeFacade(node->getUUID());
//        child_node_facade_removed(remote_node);
//    });
}


void GraphFacadeRemote::handleBroadcast(const BroadcastMessageConstPtr& message)
{
    if(auto graph_msg = std::dynamic_pointer_cast<GraphBroadcasts const>(message)) {
        switch(graph_msg->getBroadcastType()) {
        case GraphBroadcasts::GraphBroadcastType::GraphCreated:
        {
            std::cerr << "Remote graph " << graph_msg->getGraphUUID() << " has been created" << std::endl;
        }
            break;
        case GraphBroadcasts::GraphBroadcastType::GraphDestroyed:
        {
            std::cerr << "Remote graph " << graph_msg->getGraphUUID() << " has been destroyed" << std::endl;
        }
            break;
        case GraphBroadcasts::GraphBroadcastType::NodeCreated:
        {
            std::cerr << "Remote node " << graph_msg->getPayload<UUID>() << " has been created" << std::endl;
        }
            break;
        case GraphBroadcasts::GraphBroadcastType::NodeDestroyed:
        {
            std::cerr << "Remote node " << graph_msg->getPayload<UUID>() << " has been destroyed" << std::endl;
        }
            break;
        default:
            break;
        }
    }
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
    if(uuid.empty()) {
        throw std::logic_error("cannot get subgraph for empty UUID");
    }

    if(uuid.composite()) {
        GraphFacadePtr facade = children_[uuid.rootUUID()];
        return facade->getSubGraph(uuid.nestedUUID());
    } else {
        GraphFacadePtr facade = children_[uuid];
        return facade.get();
    }
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
