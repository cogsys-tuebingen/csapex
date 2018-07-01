#ifndef NOTIFICATION_MESSAGE_H
#define NOTIFICATION_MESSAGE_H

/// PROJECT
#include <csapex/io/broadcast_impl.hpp>
#include <csapex/model/notification.h>
#include <csapex/serialization/serialization_fwd.h>

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

    virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    const Notification& getNotification() const;

private:
    Notification notification;
};

}  // namespace csapex

#endif  // NOTIFICATION_MESSAGE_H
