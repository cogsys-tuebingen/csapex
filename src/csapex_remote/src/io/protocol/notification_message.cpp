/// HEADER
#include <csapex/io/protcol/notification_message.h>

/// PROJECT
#include <csapex/serialization/broadcast_message_serializer.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_BROADCAST_SERIALIZER(NotificationMessage)

using namespace csapex;

NotificationMessage::NotificationMessage(const Notification& notification) : notification(notification)
{
}

NotificationMessage::NotificationMessage()
{
}

void NotificationMessage::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    data << notification;
}

void NotificationMessage::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    data >> notification;
}

const Notification& NotificationMessage::getNotification() const
{
    return notification;
}
