/// HEADER
#include <csapex/io/protcol/command_broadcasts.h>

/// PROJECT
#include <csapex/serialization/broadcast_message_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/io/feedback.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/model/graph_facade.h>
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/command/command.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_BROADCAST_SERIALIZER(CommandBroadcasts)

using namespace csapex;

///
/// Broadcast
///
CommandBroadcasts::CommandBroadcasts()
    : broadcast_type_(CommandBroadcastType::None)
{

}

CommandBroadcasts::CommandBroadcasts(CommandBroadcastType broadcast_type)
    : broadcast_type_(broadcast_type), flag_(false)
{

}
CommandBroadcasts::CommandBroadcasts(CommandBroadcastType broadcast_type, bool flag)
    : broadcast_type_(broadcast_type), flag_(flag)
{

}

void CommandBroadcasts::serialize(SerializationBuffer &data, SemanticVersion& version) const
{
    data << broadcast_type_;
    data << flag_;
}

void CommandBroadcasts::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    data >> broadcast_type_;
    data >> flag_;
}

CommandBroadcasts::CommandBroadcastType CommandBroadcasts::getBroadcastType() const
{
    return broadcast_type_;
}

bool CommandBroadcasts::getFlag() const
{
    return flag_;
}
