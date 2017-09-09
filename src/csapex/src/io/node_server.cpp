/// HEADER
#include <csapex/io/node_server.h>

/// PROJECT
#include <csapex/model/node_facade_local.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

NodeServer::NodeServer(SessionPtr session)
    : session_(session)
{

}

NodeServer::~NodeServer()
{
}


void NodeServer::startObserving(const NodeFacadeLocalPtr &node)
{
    std::cerr << "start observing node: " << node->getAUUID() << std::endl;
}

void NodeServer::stopObserving(const NodeFacadeLocalPtr &node)
{
    std::cerr << "stop observing node: " << node->getAUUID() << std::endl;
}
