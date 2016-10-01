#ifndef NOTIFICATION_H
#define NOTIFICATION_H

/// COMPONENT
#include <csapex/utility/uuid.h>

namespace csapex {

class Notification
{
public:
    UUID uuid;
    std::string message;
};

}

#endif // NOTIFICATION_H
