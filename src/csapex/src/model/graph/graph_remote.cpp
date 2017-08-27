/// HEADER
#include <csapex/model/graph/graph_remote.h>

/// PROJECT
#include <csapex/model/graph/graph_local.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/connectable.h>
#include <csapex/model/node_facade.h>
#include <csapex/model/node_facade_remote.h>
#include <csapex/io/session.h>

using namespace csapex;

GraphRemote::GraphRemote(SessionPtr session, GraphLocal &temp_reference)
    : session_(session),
      temp_reference(temp_reference)
{
    observe(temp_reference.state_changed, state_changed);

    observe(temp_reference.connection_added, connection_added);
    observe(temp_reference.connection_removed, connection_removed);

    observe(temp_reference.vertex_added, [this](graph::VertexPtr vertex) {
        std::shared_ptr<NodeFacadeRemote> remote_node_facade = std::make_shared<NodeFacadeRemote>(
                    session_,
                    vertex->getNodeFacade()->getNodeHandle(),
                    vertex->getNodeFacade()->getNodeWorker(),
                    vertex->getNodeFacade()->getNodeRunner()
                    );
        graph::VertexPtr remote_vertex = std::make_shared<graph::Vertex>(remote_node_facade);
        remote_vertices_.push_back(remote_vertex);
        vertex_added(remote_vertex);
    });
    observe(temp_reference.vertex_removed,  [this](graph::VertexPtr vertex) {
        auto pos = std::find(remote_vertices_.begin(), remote_vertices_.end(), vertex);
        if(pos != remote_vertices_.end()) {
            vertex_removed(*pos);
            remote_vertices_.erase(pos);
        }
    });

}

GraphRemote::~GraphRemote()
{
}

void GraphRemote::resetActivity()
{
    temp_reference.resetActivity();
}

void GraphRemote::clear()
{
    temp_reference.clear();
}

void GraphRemote::addNode(NodeFacadePtr nf)
{
    temp_reference.addNode(nf);
}

std::vector<ConnectionPtr> GraphRemote::getConnections()
{
    return temp_reference.getConnections();
}

void GraphRemote::deleteNode(const UUID& uuid)
{
    temp_reference.deleteNode(uuid);
}

int GraphRemote::countNodes()
{
    return temp_reference.countNodes();
}


bool GraphRemote::addConnection(ConnectionPtr connection)
{
    return temp_reference.addConnection(connection);
}

void GraphRemote::deleteConnection(ConnectionPtr connection)
{
    temp_reference.deleteConnection(connection);
}

void GraphRemote::beginTransaction()
{
    temp_reference.beginTransaction();
}

void GraphRemote::finalizeTransaction()
{
    temp_reference.finalizeTransaction();
}

void GraphRemote::analyzeGraph()
{
    temp_reference.analyzeGraph();
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

std::vector<NodeHandle*> GraphRemote::getAllNodeHandles()
{
    return temp_reference.getAllNodeHandles();
}

ConnectablePtr GraphRemote::findConnector(const UUID &uuid)
{
    return temp_reference.findConnector(uuid);
}

ConnectablePtr GraphRemote::findConnectorNoThrow(const UUID &uuid) noexcept
{
    return temp_reference.findConnectorNoThrow(uuid);
}

ConnectionPtr GraphRemote::getConnectionWithId(int id)
{
    return temp_reference.getConnectionWithId(id);
}

ConnectionPtr GraphRemote::getConnection(Connectable* from, Connectable* to)
{
    return getConnection(from->getUUID(), to->getUUID());
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
