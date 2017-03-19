#ifndef FEEDBACK_SERIALIZER_H
#define FEEDBACK_SERIALIZER_H

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/io/broadcast_message.h>

/// SYSTEM
#include <inttypes.h>

namespace csapex
{

class FeedbackSerializer : public Singleton<FeedbackSerializer>, public Serializer
{
public:
    void serialize(const SerializableConstPtr& packet, SerializationBuffer &data) override;
    SerializablePtr deserialize(SerializationBuffer &data) override;
};

}

#endif // FEEDBACK_SERIALIZER_H
