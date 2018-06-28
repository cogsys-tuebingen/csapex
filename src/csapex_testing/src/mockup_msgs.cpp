/// HEADER
#include <csapex_testing/mockup_msgs.h>

/// PROJECT
#include <csapex/utility/register_msg.h>

using namespace csapex;

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::MockMessage)
CSAPEX_REGISTER_MESSAGE(csapex::connection_types::VectorMessage)
CSAPEX_REGISTER_MESSAGE(csapex::connection_types::BaseMessage)
CSAPEX_REGISTER_MESSAGE(csapex::connection_types::ChildMessage)

SerializationBuffer& csapex::operator << (SerializationBuffer& data, const Mock& t)
{
    data << t.payload;
    return data;
}

const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, Mock& t)
{
    data >> t.payload;
    return data;
}


SerializationBuffer& csapex::operator << (SerializationBuffer& data, const Foo& t)
{
    data << t.value;
    return data;
}

const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, Foo& t)
{
    data >> t.value;
    return data;
}



SerializationBuffer& csapex::operator << (SerializationBuffer& data, const Base& t)
{
    return data;
}

const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, Base& t)
{
    return data;
}


SerializationBuffer& csapex::operator << (SerializationBuffer& data, const Child& t)
{
    return data;
}

const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, Child& t)
{
    return data;
}
