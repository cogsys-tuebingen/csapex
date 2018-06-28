/// HEADER
#include <csapex/serialization/serializable.h>

/// PROJECT
#include <csapex/serialization/io/csapex_io.h>

/// SYSTEM
#include <inttypes.h>
#include <string>
#include <ostream>

using namespace csapex;

Serializable::~Serializable()
{

}

SemanticVersion Serializable::getVersion() const
{
    return SemanticVersion();
}

void Serializable::serializeVersioned(SerializationBuffer &data) const
{
    SemanticVersion version = getVersion();
    SemanticVersion v = version;

    auto start_pos = data.size();
    data << version;

    serialize(data, version);

    if(v != version) {
        // if an implementation overwrites the version, update the serialized result
        SerializationBuffer tmp;
        tmp << version;
        for(auto it = tmp.begin() + SerializationBuffer::HEADER_LENGTH; it != tmp.end(); ++it, ++start_pos) {
            data.at(start_pos) = *it;
        }
    }
}
void Serializable::deserializeVersioned(const SerializationBuffer& data)
{
    SemanticVersion version;
    data >> version;
    deserialize(data, version);
}
