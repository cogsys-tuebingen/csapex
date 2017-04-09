#ifndef REGISTER_MSG_H
#define REGISTER_MSG_H

/// SYSTEM
#include <csapex/factory/message_factory.h>
#include <csapex/serialization/message_serializer.h>
#include <csapex/msg/generic_vector_message.hpp>

#define CSAPEX_REGISTER_MESSAGE_WITH_NAME(name,instancename)\
namespace csapex { \
namespace connection_types { \
    static MessageConstructorRegistered<name> instancename##_c; \
    static MessageSerializerRegistered<name> instancename##_s; \
    static GenericVectorRegistered<name> instancename##_gv; \
} \
}
#define CSAPEX_REGISTER_MESSAGE(name)\
    CSAPEX_REGISTER_MESSAGE_WITH_NAME(name,g_instance_)

#endif // REGISTER_MSG_H
