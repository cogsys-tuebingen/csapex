/// HEADER
#include <csapex/model/graph/graph_remote.h>

/// PROJECT
#include <csapex/model/graph/graph_local.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/connectable.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_facade.h>
#include <csapex/model/node_facade_remote.h>
#include <csapex/io/session.h>

using namespace csapex;

GraphRemote::GraphRemote(SessionPtr session, const AUUID& auuid,
                         GraphLocal &temp_reference)
    : session_(session),
      temp_reference(temp_reference),
      nf_(std::make_shared<NodeFacadeRemote>(session, auuid, nullptr, nullptr, nullptr))
{
    observe(temp_reference.state_changed, state_changed);

    observe(temp_reference.connection_added, connection_added);
    observe(temp_reference.connection_removed, connection_removed);

    observe(temp_reference.vertex_added, [this](graph::VertexPtr vertex) {
        std::shared_ptr<NodeFacadeRemote> remote_node_facade = std::make_shared<NodeFacadeRemote>(
                    session_,
                    vertex->getNodeFacade()->getAUUID(),
                    vertex->getNodeFacade()->getNodeHandle(),
                    vertex->getNodeFacade()->getNodeWorker(),
                    vertex->getNodeFacade()->getNodeHandle()->getNodeRunner()
                    );
        graph::VertexPtr remote_vertex = std::make_shared<graph::Vertex>(remote_node_facade);
        remote_vertices_.push_back(remote_vertex);
        vertex_added(remote_vertex);
    });
    observe(temp_reference.vertex_removed,  [this](graph::VertexPtr vertex) {
        for(auto it = remote_vertices_.begin() ; it != remote_vertices_.end(); ++it) {
            graph::VertexPtr remote_vertex = *it;
            if(remote_vertex->getNodeFacade()->getUUID() == vertex->getNodeFacade()->getUUID()) {
                vertex_removed(remote_vertex);
                remote_vertices_.erase(it);
                return;
            }
        }
    });

}

GraphRemote::~GraphRemote()
{
}

AUUID GraphRemote::getAbsoluteUUID() const
{
    return nf_->getUUID().getAbsoluteUUID();
}


void GraphRemote::resetActivity()
{
    temp_reference.resetActivity();
}

void GraphRemote::clear()
{
    temp_reference.clear();
}

std::vector<ConnectionPtr> GraphRemote::getConnections()
{
    return temp_reference.getConnections();
}

int GraphRemote::countNodes()
{
    return temp_reference.countNodes();
}


int GraphRemote::getComponent(const UUID &node_uuid) const
{
    return temp_reference.getComponent(node_uuid);
}

int GraphRemote::getDepth(const UUID &node_uuid) const
{
    return temp_reference.getDepth(node_uuid);
}

Node* GraphRemote::findNode(const UUID& uuid) const
{
    return temp_reference.findNode(uuid);
}



NodeHandle* GraphRemote::findNodeHandle(const UUID& uuid) const
{
    return temp_reference.findNodeHandle(uuid);
}

Node* GraphRemote::findNodeNoThrow(const UUID& uuid) const noexcept
{
    return temp_reference.findNodeNoThrow(uuid);
}


NodeHandle *GraphRemote::findNodeHandleNoThrow(const UUID& uuid) const noexcept
{
    return temp_reference.findNodeHandleNoThrow(uuid);
}

Node* GraphRemote::findNodeForConnector(const UUID &uuid) const
{
    return findNodeForConnector(uuid);
}


NodeHandle* GraphRemote::findNodeHandleForConnector(const UUID &uuid) const
{
    return temp_reference.findNodeHandleForConnector(uuid);
}

NodeHandle* GraphRemote::findNodeHandleForConnectorNoThrow(const UUID &uuid) const noexcept
{
    return findNodeHandleNoThrow(uuid.parentUUID());
}

NodeHandle* GraphRemote::findNodeHandleWithLabel(const std::string& label) const
{
    return temp_reference.findNodeHandleWithLabel(label);
}


NodeFacadePtr GraphRemote::findNodeFacade(const UUID& uuid) const
{
    if(uuid.empty()) {
        return nf_;
    }
    NodeFacadePtr node_facade = findNodeFacadeNoThrow(uuid);
    if(node_facade) {
        return node_facade;
    }
    throw NodeFacadeNotFoundException(uuid.getFullName());
}
NodeFacadePtr GraphRemote::findNodeFacadeNoThrow(const UUID& uuid) const noexcept
{
    if(uuid.empty()) {
        return nf_;
    }
    if(uuid.composite()) {
        // TODO: implement for C/S
        return temp_reference.findNodeFacade(uuid);
//        UUID root = uuid.rootUUID();

//        NodeFacadePtr root_nf = findNodeFacadeNoThrow(root);
//        if(root_nf) {
//            NodePtr root_node = root_nf->getNodeHandle()->getNode().lock();
//            if(root_node) {
//                SubgraphNodePtr graph = std::dynamic_pointer_cast<SubgraphNode>(root_node);
//                if(graph) {
//                    return graph->findNodeFacade(uuid.nestedUUID());
//                }
//            }
//        }

    } else {
        for(const auto vertex : remote_vertices_) {
            NodeFacadePtr nf = vertex->getNodeFacade();
            if(nf->getUUID() == uuid) {
                return nf;
            }
        }
    }

    return nullptr;
}
NodeFacadePtr GraphRemote::findNodeFacadeForConnector(const UUID &uuid) const
{
    try {
        return findNodeFacade(uuid.parentUUID());

    } catch(const std::exception& e) {
        throw std::runtime_error(std::string("cannot find handle of connector \"") + uuid.getFullName());
    }
}
NodeFacadePtr GraphRemote::findNodeFacadeForConnectorNoThrow(const UUID &uuid) const noexcept
{
    return findNodeFacadeNoThrow(uuid.parentUUID());
}
NodeFacadePtr GraphRemote::findNodeFacadeWithLabel(const std::string& label) const
{
    for(const auto vertex : remote_vertices_) {
        NodeFacadePtr nf = vertex->getNodeFacade();
        if(nf->getLabel() == label) {
            return nf;
        }
    }

    return nullptr;
}


Graph* GraphRemote::findSubgraph(const UUID &uuid) const
{
    return temp_reference.findSubgraph(uuid);
}

std::vector<NodeHandle*> GraphRemote::getAllNodeHandles()
{
    return temp_reference.getAllNodeHandles();
}
std::vector<NodeFacadePtr> GraphRemote::getAllNodeFacades()
{
    return temp_reference.getAllNodeFacades();
}

ConnectorPtr GraphRemote::findConnector(const UUID &uuid)
{
    ConnectorPtr res = findConnectorNoThrow(uuid);
    if(!res) {
        throw std::runtime_error(std::string("cannot find connector with UUID=") + uuid.getFullName());
    }
    return res;
}

ConnectorPtr GraphRemote::findConnectorNoThrow(const UUID &uuid) noexcept
{
    NodeFacadePtr owner = findNodeFacadeNoThrow(uuid.parentUUID());
    if(!owner) {
        return nullptr;
    }
    NodeFacadeRemotePtr owner_remote = std::dynamic_pointer_cast<NodeFacadeRemote>(owner);
    apex_assert_hard(owner_remote);

    try {
        return owner->getConnector(uuid);
    } catch(...) {
        return nullptr;
    }
}

ConnectionPtr GraphRemote::getConnectionWithId(int id)
{
    return temp_reference.getConnectionWithId(id);
}

ConnectionPtr GraphRemote::getConnection(const UUID &from, const UUID &to)
{
    return temp_reference.getConnection(from, to);
}

Graph::vertex_iterator GraphRemote::begin()
{
    return temp_reference.begin();
}

const Graph::vertex_const_iterator GraphRemote::begin() const
{
    return temp_reference.begin();
}


Graph::vertex_iterator GraphRemote::end()
{
    return temp_reference.end();
}

const Graph::vertex_const_iterator GraphRemote::end() const
{
    return temp_reference.end();
}
