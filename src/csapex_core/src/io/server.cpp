/// HEADER
#include <csapex/io/server.h>

/// COMPONENT
#include <csapex/core/csapex_core.h>

using namespace csapex;

Server::Server(CsApexCore& core) : core_(core)
{
    observe(core_.shutdown_requested, [this]() { stop(); });
}

Server::~Server()
{
}
