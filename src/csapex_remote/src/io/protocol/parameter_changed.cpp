/// HEADER
#include <csapex/io/protcol/parameter_changed.h>

/// PROJECT
#include <csapex/serialization/broadcast_message_serializer.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/serialization/io/std_io.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_BROADCAST_SERIALIZER(ParameterChanged)

using namespace csapex;

ParameterChanged::ParameterChanged(const UUID &id, const boost::any &value)
    : uuid(id), value(value)
{

}

ParameterChanged::ParameterChanged()
{

}

void ParameterChanged::serialize(SerializationBuffer &data) const
{
    data << uuid;
    data << value;
}

void ParameterChanged::deserialize(const SerializationBuffer& data)
{
    data >> uuid;
    data >> value;
}


AUUID ParameterChanged::getUUID() const
{
    return uuid;
}

boost::any ParameterChanged::getValue() const
{
    return value;
}
