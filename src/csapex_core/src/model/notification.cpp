/// HEADER
#include <csapex/model/notification.h>

/// PROJECT
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>

using namespace csapex;

Notification::Notification(const Notification& copy)
    : auuid(copy.auuid), message(copy.message.str()), error(copy.error), msg_dirty_(true)
{}
void Notification::operator = (const Notification& copy)
{
    auuid = copy.auuid;
    error = copy.error;

    msg_cache_ = copy.getMessage();
    message.str(msg_cache_);
    msg_dirty_ = false;
}

Notification::Notification(const std::string& message)
    : Notification(AUUID(), message, ErrorState::ErrorLevel::ERROR)
{}
Notification::Notification(AUUID uuid, const std::string& message)
    : Notification(uuid, message, ErrorState::ErrorLevel::ERROR)
{}
Notification::Notification(AUUID uuid, const std::string& message, ErrorState::ErrorLevel error)
    : auuid(uuid), message(message), error(error), msg_dirty_(false), msg_cache_(message)
{}

void Notification::serialize(SerializationBuffer &data, SemanticVersion& version) const
{
    data << auuid;
    data << message;
    data << error;
}

void Notification::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    data >> auuid;
    data >> message;
    data >> error;
}

bool Notification::operator == (const Notification& other) const
{
    return auuid == other.auuid &&
            error == other.error &&
            getMessage() == other.getMessage();
}

std::string Notification::getMessage() const
{
    if(msg_dirty_) {
        msg_cache_ = message.str();
        msg_dirty_ = false;
    }

    return msg_cache_;
}
