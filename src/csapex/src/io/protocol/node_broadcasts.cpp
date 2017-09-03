/// HEADER
#include <csapex/io/protcol/node_broadcasts.h>

/// PROJECT
#include <csapex/serialization/broadcast_message_serializer.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/io/feedback.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/model/graph_facade.h>
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/command/command.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_BROADCAST_SERIALIZER(NodeBroadcasts)

using namespace csapex;

///
/// Broadcast
///
NodeBroadcasts::NodeBroadcasts()
    : broadcast_type_(NodeBroadcastType::None)
{

}

NodeBroadcasts::NodeBroadcasts(NodeBroadcastType broadcast_type)
    : broadcast_type_(broadcast_type)
{

}
NodeBroadcasts::NodeBroadcasts(NodeBroadcastType broadcast_type, AUUID uuid)
    : broadcast_type_(broadcast_type), uuid_(uuid)
{

}

void NodeBroadcasts::serialize(SerializationBuffer &data) const
{
    data << broadcast_type_;
    data << uuid_;
}

void NodeBroadcasts::deserialize(SerializationBuffer& data)
{
    data >> broadcast_type_;
    data >> uuid_;
}

NodeBroadcasts::NodeBroadcastType NodeBroadcasts::getBroadcastType() const
{
    return broadcast_type_;
}

AUUID NodeBroadcasts::getUUID() const
{
    return uuid_;
}
