#ifndef REGISTER_MSG_H
#define REGISTER_MSG_H

/// SYSTEM
#include <csapex/msg/message_factory.h>

#define CSAPEX_REGISTER_MESSAGE(name)\
namespace csapex { \
namespace connection_types { \
template <> \
struct MessageRegistered<name> \
{ \
    MessageRegistered() { \
        csapex::MessageFactory::registerMessage<name>(); \
    } \
}; \
static MessageRegistered<name> g_instance_; \
} \
}

#endif // REGISTER_MSG_H
