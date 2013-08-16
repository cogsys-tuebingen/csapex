/// HEADER
#include <csapex/connector_out_forward.h>

using namespace csapex;

ConnectorOutForward::ConnectorOutForward(Box* parent, const std::string& uuid)
    : ConnectorOut(parent, uuid)
{
}

ConnectorOutForward::ConnectorOutForward(Box* parent, int sub_id)
    : ConnectorOut(parent, sub_id)
{
}

bool ConnectorOutForward::isForwarding() const
{
    return true;
}
