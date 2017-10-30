/// HEADER
#include <csapex/io/remote.h>

using namespace csapex;

Remote::Remote(const SessionPtr &session)
    : session_(session)
{
    broadcast_connection_ = session_->broadcast_received.connect(this, &Remote::handleBroadcast);
}

Remote::~Remote()
{

}

void Remote::handleBroadcast(const BroadcastMessageConstPtr &message)
{

}
