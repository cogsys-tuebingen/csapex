/// HEADER
#include <csapex/io/node_server.h>

/// PROJECT
#include <csapex/model/connectable.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/model/node_handle.h>
#include <csapex/io/session.h>
#include <csapex/io/connector_server.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

NodeServer::NodeServer(SessionPtr session)
    : session_(session)
{
    connector_server_ = std::make_shared<ConnectorServer>(session_);
}

NodeServer::~NodeServer()
{
}


void NodeServer::startObserving(const NodeFacadeLocalPtr &node)
{
    std::cerr << "start serving node: " << node->getAUUID() << " with " << node->getExternalConnectors().size() << " connectors" << std::endl;
    io::ChannelPtr channel = session_->openChannel(node->getAUUID());

    for(const ConnectorDescription& cd : node->getExternalConnectors()) {
        ConnectorPtr c = node->getConnector(cd.id);
        connector_server_->startObserving(std::dynamic_pointer_cast<Connectable>(c));
    }

    channels_[node->getAUUID()] = channel;
}

void NodeServer::stopObserving(const NodeFacadeLocalPtr &node)
{
    std::cerr << "stop serving node: " << node->getAUUID() << std::endl;

    auto pos = channels_.find(node->getAUUID());
    if(pos != channels_.end()) {
        channels_.erase(pos);
    }
}
