/// HEADER
#include <csapex/model/node_handle.h>

using namespace csapex;

NodeHandle::NodeHandle(const std::string &type, const UUID& uuid, NodePtr node)
    :  Unique(uuid), node_type_(type), node_(node)
{

}

NodeHandle::~NodeHandle()
{

}

std::string NodeHandle::getType() const
{
    return node_type_;
}

NodeWeakPtr NodeHandle::getNode() const
{
    return node_;
}
