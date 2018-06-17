#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/core/settings/settings_impl.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/input.h>
#include <csapex/msg/message_template.hpp>
#include <csapex/msg/direct_connection.h>
#include <csapex/msg/static_output.h>
#include <csapex/msg/input.h>
#include <csapex/utility/register_msg.h>

#include <csapex_testing/stepping_test.h>
#include <csapex_testing/mockup_msgs.h>

#include <csapex_testing/csapex_test_case.h>


namespace csapex
{

namespace connection_types
{
class MockSerializationMessage : public MessageTemplate<Mock, MockSerializationMessage>
{
public:
//    SemanticVersion getVersion() const
//    {
//        return version_;
//    }

    void serialize(SerializationBuffer &data, SemanticVersion& version) const override
    {
        version = version_;

        MessageTemplate<Mock, MockSerializationMessage>::serialize(data, version);
    }
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override
    {
        if(version.major_v == 123) {
            value.payload = "this is version 123";
        } else {
            MessageTemplate<Mock, MockSerializationMessage>::deserialize(data, version);
        }
        version_ = version;
    }

    SemanticVersion version_;
};
}

class MessageSerializationTest : public CsApexTestCase
{
public:
    MessageSerializationTest()
    {

    }

    virtual ~MessageSerializationTest() {
        // You can do clean-up work that doesn't throw exceptions here.
    }

};

namespace connection_types
{

template <>
struct type<MockSerializationMessage> {
    static std::string name() {
        return "MockSerializationMessage";
    }
};

}

}

namespace YAML {
template<>
struct convert<csapex::connection_types::MockSerializationMessage>
{
    static Node encode(const csapex::connection_types::MockSerializationMessage& rhs)
    {
        Node n;
        return n;
    }

    static bool decode(const Node& node, csapex::connection_types::MockSerializationMessage& rhs)
    {
        return true;
    }
};
}

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::MockSerializationMessage)

namespace csapex
{


TEST_F(MessageSerializationTest, LoadAnyVersion)
{
    csapex::SerializationBuffer data;
    {
        connection_types::MockSerializationMessage::Ptr message(new connection_types::MockSerializationMessage);
        message->version_.major_v = 1;
        message->version_.minor_v = 2;
        message->version_.patch_v = 3;
        message->value.payload = "will not be ignored";
        data << message;
    }
    {
        connection_types::MockSerializationMessage::Ptr message;
        data >> message;
        ASSERT_EQ(1, message->version_.major_v);
        ASSERT_EQ(2, message->version_.minor_v);
        ASSERT_EQ(3, message->version_.patch_v);
        ASSERT_STREQ("will not be ignored", message->value.payload.c_str());
    }
}



TEST_F(MessageSerializationTest, LoadSpecificVersion)
{
    csapex::SerializationBuffer data;
    {
        connection_types::MockSerializationMessage::Ptr message(new connection_types::MockSerializationMessage);
        message->version_.major_v = 123;
        message->version_.minor_v = 0;
        message->version_.patch_v = 0;
        message->value.payload = "will be ignored";
        data << message;
    }
    {
        connection_types::MockSerializationMessage::Ptr message;
        data >> message;
        ASSERT_EQ(123, message->version_.major_v);
        ASSERT_EQ(0, message->version_.minor_v);
        ASSERT_EQ(0, message->version_.patch_v);
        ASSERT_STREQ("this is version 123", message->value.payload.c_str());
    }
}



}
