/// HEADER
#include <csapex/io/proxy.h>

using namespace csapex;

Proxy::Proxy(const SessionPtr &session)
    : session_(session)
{
    broadcast_connection_ = session_->broadcast_received.connect(this, &Proxy::handleBroadcast);
}

Proxy::~Proxy()
{

}

void Proxy::handleBroadcast(const BroadcastMessageConstPtr &message)
{

}
