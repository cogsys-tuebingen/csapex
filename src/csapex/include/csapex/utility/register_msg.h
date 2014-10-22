#ifndef REGISTER_MSG_H
#define REGISTER_MSG_H

/// SYSTEM
#include <csapex/msg/message_factory.h>

#define CSAPEX_REGISTER_MESSAGE_WITH_NAME(name,instancename)\
namespace csapex { \
namespace connection_types { \
static MessageRegistered<name> instancename; \
} \
}
#define CSAPEX_REGISTER_MESSAGE(name)\
    CSAPEX_REGISTER_MESSAGE_WITH_NAME(name,g_instance_)

#endif // REGISTER_MSG_H
