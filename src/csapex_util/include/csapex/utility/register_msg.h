#ifndef REGISTER_MSG_H
#define REGISTER_MSG_H

/// SYSTEM
#include <csapex/factory/message_factory.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>
#include <csapex/serialization/message_serializer.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>

#define MESSAGE_CONCATENATE_DETAIL(x,y) x##y
#define MESSAGE_CONCATENATE(x,y) MESSAGE_CONCATENATE_DETAIL(x,y)
#define MESSAGE_MAKE_UNIQUE(x) MESSAGE_CONCATENATE(x,__LINE__)

#define CSAPEX_REGISTER_MESSAGE_WITH_NAME(name,instancename)\
namespace csapex { \
namespace connection_types { \
    static MessageConstructorRegistered<name> MESSAGE_CONCATENATE(c_,instancename); \
    static MessageSerializerRegistered<name> MESSAGE_CONCATENATE(s_,instancename); \
    static GenericVectorRegistered<name> MESSAGE_CONCATENATE(gv_,instancename); \
} \
}

#define CSAPEX_REGISTER_MESSAGE(name)\
    CSAPEX_REGISTER_MESSAGE_WITH_NAME(name, MESSAGE_MAKE_UNIQUE(g_instance) )

#endif // REGISTER_MSG_H
