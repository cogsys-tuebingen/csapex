/// HEADER
#include <csapex/io/note.h>

/// PROJECT
#include <csapex/serialization/serialization_buffer.h>

using namespace csapex;
using namespace csapex::io;

uint8_t Note::getPacketType() const
{
    return PACKET_TYPE_ID;
}

Note::Note(AUUID uuid)
    : uuid_(uuid)
{

}

AUUID Note::getAUUID() const
{
    return uuid_;
}

void Note::serialize(SerializationBuffer &data) const
{
    data << uuid_;
}

void Note::deserialize(SerializationBuffer& data)
{
    data >> uuid_;
}
