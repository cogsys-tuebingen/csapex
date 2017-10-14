/// HEADER
#include <csapex/utility/notification.h>

/// PROJECT
#include <csapex/serialization/serialization_buffer.h>

using namespace csapex;

Notification::Notification(const Notification& copy)
    : auuid(copy.auuid), message(copy.message.str()), error(copy.error)
{}
void Notification::operator = (const Notification& copy)
{
    auuid = copy.auuid;
    message.str(copy.message.str());
    error = copy.error;
}

Notification::Notification(const std::string& message)
    : Notification(AUUID(), message, ErrorState::ErrorLevel::ERROR)
{}
Notification::Notification(AUUID uuid, const std::string& message)
    : Notification(uuid, message, ErrorState::ErrorLevel::ERROR)
{}
Notification::Notification(AUUID uuid, const std::string& message, ErrorState::ErrorLevel error)
    : auuid(uuid), message(message),error(error)
{}

void Notification::serialize(SerializationBuffer &data) const
{
    data << auuid;
    data << message;
    data << error;
}

void Notification::deserialize(SerializationBuffer& data)
{
    data >> auuid;
    data >> message;
    data >> error;
}

std::shared_ptr<Clonable> Notification::makeEmptyClone() const
{
    return std::shared_ptr<Notification>(new Notification);
}
