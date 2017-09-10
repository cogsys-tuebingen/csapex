/// HEADER
#include <csapex/io/connector_server.h>

/// PROJECT
#include <csapex/model/node_facade_local.h>
#include <csapex/model/node_handle.h>
#include <csapex/io/session.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

ConnectorServer::ConnectorServer(SessionPtr session)
    : session_(session)
{

}

ConnectorServer::~ConnectorServer()
{
}


void ConnectorServer::startObserving(const ConnectablePtr &connector)
{
    std::cerr << "start serving connector: " << connector->getAUUID()  << std::endl;
    io::ChannelPtr channel = session_->openChannel(connector->getAUUID());

    channels_[connector->getAUUID()] = channel;
}

void ConnectorServer::stopObserving(const ConnectablePtr &connector)
{
    std::cerr << "stop serving connector: " << connector->getAUUID() << std::endl;

    auto pos = channels_.find(connector->getAUUID());
    if(pos != channels_.end()) {
        channels_.erase(pos);
    }
}
