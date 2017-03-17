#ifndef NOTIFICATION_MESSAGE_H
#define NOTIFICATION_MESSAGE_H

/// PROJECT
#include <csapex/io/broadcast_message.h>
#include <csapex/utility/notification.h>
#include <csapex/serialization/serialization_buffer.h>

namespace csapex
{

namespace io
{
    class NotificationMessageSerializer;
}

class NotificationMessage : public BroadcastMessage
{
public:
    NotificationMessage(const Notification& notification);
    NotificationMessage();
    virtual std::string getType() const override;

    virtual void serialize(SerializationBuffer &data) override;
    virtual void deserialize(SerializationBuffer& data) override;

    const Notification& getNotification() const;

private:
    Notification notification;
};

}

#endif // NOTIFICATION_MESSAGE_H
