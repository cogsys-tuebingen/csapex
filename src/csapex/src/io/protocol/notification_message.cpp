/// HEADER
#include <csapex/io/protcol/notification_message.h>

/// PROJECT
#include <csapex/serialization/broadcast_message_serializer.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_BROADCAST_SERIALIZER(NotificationMessage)

using namespace csapex;

NotificationMessage::NotificationMessage(const Notification &notification)
    : notification(notification)
{

}

NotificationMessage::NotificationMessage()
{

}

void NotificationMessage::serialize(SerializationBuffer &data) const
{
    data << notification;
}

void NotificationMessage::deserialize(SerializationBuffer& data)
{
    data >> notification;
}

const Notification& NotificationMessage::getNotification() const
{
    return notification;
}
