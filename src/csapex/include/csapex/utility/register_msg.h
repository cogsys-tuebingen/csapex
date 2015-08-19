#ifndef REGISTER_MSG_H
#define REGISTER_MSG_H

/// SYSTEM
#include <csapex/factory/message_factory.h>
#include <csapex/serialization/message_serializer.h>

#define CSAPEX_REGISTER_MESSAGE_WITH_NAME(name,instancename)\
namespace csapex { \
namespace connection_types { \
    static MessageConstructorRegistered<name> instancename##_c; \
    static MessageSerializerRegistered<name> instancename##_s; \
} \
}
#define CSAPEX_REGISTER_MESSAGE(name)\
    CSAPEX_REGISTER_MESSAGE_WITH_NAME(name,g_instance_)

#endif // REGISTER_MSG_H
