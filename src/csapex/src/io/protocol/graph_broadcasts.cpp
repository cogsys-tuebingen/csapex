/// HEADER
#include <csapex/io/protcol/graph_broadcasts.h>

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

CSAPEX_REGISTER_BROADCAST_SERIALIZER(GraphBroadcasts)

using namespace csapex;

///
/// Broadcast
///
GraphBroadcasts::GraphBroadcasts()
    : broadcast_type_(GraphBroadcastType::None)
{

}

GraphBroadcasts::GraphBroadcasts(GraphBroadcastType broadcast_type)
    : broadcast_type_(broadcast_type)
{

}
GraphBroadcasts::GraphBroadcasts(GraphBroadcastType broadcast_type, AUUID uuid)
    : broadcast_type_(broadcast_type), graph_uuid_(uuid)
{

}
GraphBroadcasts::GraphBroadcasts(GraphBroadcastType broadcast_type, AUUID uuid, boost::any payload)
    : broadcast_type_(broadcast_type), graph_uuid_(uuid), payload_(payload)
{

}

void GraphBroadcasts::serialize(SerializationBuffer &data) const
{
    data << broadcast_type_;
    data << graph_uuid_;
    data << payload_;
}

void GraphBroadcasts::deserialize(SerializationBuffer& data)
{
    data >> broadcast_type_;
    data >> graph_uuid_;
    data >> payload_;
}

GraphBroadcasts::GraphBroadcastType GraphBroadcasts::getBroadcastType() const
{
    return broadcast_type_;
}

AUUID GraphBroadcasts::getGraphUUID() const
{
    return graph_uuid_;
}
