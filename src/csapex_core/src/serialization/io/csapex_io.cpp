/// HEADER
#include <csapex/serialization/io/csapex_io.h>

/// COMPONENT
#include <csapex/serialization/io/std_io.h>

/// PROJECT
#include <csapex/utility/uuid.h>

using namespace csapex;

// base
SerializationBuffer& csapex::operator << (SerializationBuffer& data, const Serializable& s)
{
    s.serialize(data);
    return data;
}

const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, Serializable& s)
{
    s.deserialize(data);
    return data;
}

// UUID
SerializationBuffer& csapex::operator << (SerializationBuffer& data, const UUID& s)
{
    data << s.getFullName();
    return data;
}

const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, UUID& s)
{
    std::string full_name;
    data >> full_name;
    s = UUIDProvider::makeUUID_without_parent(full_name);
    return data;
}
