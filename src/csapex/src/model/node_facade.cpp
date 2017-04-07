/// HEADER
#include <csapex/model/node_facade.h>

using namespace csapex;

NodeFacade::NodeFacade(NodeHandleWeakPtr nh, NodeWorkerWeakPtr nw)
    : nh_(nh), nw_(nw)
{

}

NodeFacade::NodeFacade(NodeHandleWeakPtr nh)
    : nh_(nh)
{

}


NodeWorkerPtr NodeFacade::getNodeWorker()
{
    return nw_.lock();
}

NodeHandlePtr NodeFacade::getNodeHandle()
{
    return nh_.lock();
}
