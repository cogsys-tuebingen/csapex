#ifndef NOTIFICATION_MESSAGE_H
#define NOTIFICATION_MESSAGE_H

/// PROJECT
#include <csapex/io/broadcast_impl.hpp>
#include <csapex/utility/notification.h>
#include <csapex/serialization/serialization_buffer.h>

namespace csapex
{

namespace io
{
    class NotificationMessageSerializer;
}

class NotificationMessage : public BroadcastImplementation<NotificationMessage>
{
public:
    NotificationMessage(const Notification& notification);
    NotificationMessage();

    virtual void serialize(SerializationBuffer &data) const override;
    virtual void deserialize(SerializationBuffer& data) override;

    const Notification& getNotification() const;

private:
    Notification notification;
};

}

#endif // NOTIFICATION_MESSAGE_H
