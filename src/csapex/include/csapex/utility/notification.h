#ifndef NOTIFICATION_H
#define NOTIFICATION_H

/// COMPONENT
#include <csapex/utility/uuid.h>
#include <csapex/model/error_state.h>

/// SYSTEM
#include <sstream>

namespace csapex {

class Notification
{
public:
    AUUID auuid;
    std::stringstream message;
    ErrorState::ErrorLevel error;

    Notification() = default;

    Notification(const Notification& copy)
        : auuid(copy.auuid), message(copy.message.str()), error(copy.error)
    {}
    void operator = (const Notification& copy)
    {
        auuid = copy.auuid;
        message.str(copy.message.str());
        error = copy.error;
    }

    Notification(const std::string& message)
        : Notification(AUUID(), message, ErrorState::ErrorLevel::ERROR)
    {}
    Notification(AUUID uuid, const std::string& message)
        : Notification(uuid, message, ErrorState::ErrorLevel::ERROR)
    {}
    Notification(AUUID uuid, const std::string& message, ErrorState::ErrorLevel error)
        : auuid(uuid), message(message),error(error)
    {}

    template <typename T>
    Notification& operator << (const T& val)
    {
        message << val;
        return *this;
    }
};

}

#endif // NOTIFICATION_H
