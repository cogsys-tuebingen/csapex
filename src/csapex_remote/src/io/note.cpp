/// HEADER
#include <csapex/io/note.h>

/// PROJECT
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>

using namespace csapex;
using namespace csapex::io;

uint8_t Note::getPacketType() const
{
    return PACKET_TYPE_ID;
}

Note::Note(AUUID uuid) : uuid_(uuid)
{
}

AUUID Note::getAUUID() const
{
    return uuid_;
}

void Note::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    data << uuid_;
}

void Note::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    data >> uuid_;
}
