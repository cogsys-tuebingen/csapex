/// HEADER
#include <csapex/io/protcol/notification_message.h>

/// PROJECT
#include <csapex/serialization/broadcast_message_serializer.h>
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <iostream>


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
//    std::cerr << "serializing Notification" << std::endl;
    data << notification.auuid.getFullName();

    data << notification.error;
    data << notification.message;
}

void NotificationMessage::deserialize(SerializationBuffer& data)
{
//    std::cerr << "deserializing Notification" << std::endl;

    std::string full_name;
    data >> full_name;
    notification.auuid = AUUID(UUIDProvider::makeUUID_without_parent(full_name));

//    std::cerr << "full name: " << full_name << std::endl;

    data >> notification.error;
//    std::cerr << "error level: " << (int) notification.error << std::endl;

    data >> notification.message;
//    std::cerr << "message: " << notification.message.str() << std::endl;
}

const Notification& NotificationMessage::getNotification() const
{
    return notification;
}

namespace csapex
{
namespace io
{

class NotificationMessageSerializer : public BroadcastMessageSerializerInterface
{
    virtual void serialize(const BroadcastMessageConstPtr& packet, SerializationBuffer &data) override
    {
        packet->serialize(data);
    }
    virtual BroadcastMessagePtr deserialize(SerializationBuffer& data) override
    {
        auto result = std::make_shared<NotificationMessage>();
        result->deserialize(data);
        return result;
    }
};
}
BroadcastMessageSerializerRegistered<io::NotificationMessageSerializer> g_register_broadcast_message_notification_(NotificationMessage::typeName());
}

