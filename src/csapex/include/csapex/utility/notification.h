#ifndef NOTIFICATION_H
#define NOTIFICATION_H

/// COMPONENT
#include <csapex/utility/uuid.h>
#include <csapex/model/error_state.h>

namespace csapex {

class Notification
{
public:
    AUUID auuid;
    std::string message;

    ErrorState::ErrorLevel error;
};

}

#endif // NOTIFICATION_H
