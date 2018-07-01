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
class VersionedMockV1
{
public:
    int32_t payload;
};

template <>
struct semantic_version<VersionedMockV1>
{
    static constexpr SemanticVersion value{ 1, 0, 0 };
};

SerializationBuffer& operator<<(SerializationBuffer& data, VersionedMockV1& t)
{
    data << t.payload;
    return data;
}
const SerializationBuffer& operator>>(const SerializationBuffer& data, VersionedMockV1& t)
{
    data >> t.payload;
    return data;
}

class VersionedMockV2
{
public:
    std::string payload;
};

template <>
struct semantic_version<VersionedMockV2>
{
    static constexpr SemanticVersion value{ 2, 0, 0 };
};

SerializationBuffer& operator<<(SerializationBuffer& data, VersionedMockV2& t)
{
    data << t.payload;
    return data;
}
const SerializationBuffer& operator>>(const SerializationBuffer& data, VersionedMockV2& t)
{
    data >> t.payload;
    return data;
}

namespace connection_types
{
class VersionedMockSerializationMessage1 : public MessageTemplate<VersionedMockV1, VersionedMockSerializationMessage1>
{
public:
    void serialize(SerializationBuffer& data, SemanticVersion& version) const override
    {
        MessageTemplate<VersionedMockV1, VersionedMockSerializationMessage1>::serialize(data, version);
    }
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override
    {
        if (version.major_v == 123) {
            value.payload = 123;
        } else {
            MessageTemplate<VersionedMockV1, VersionedMockSerializationMessage1>::deserialize(data, version);
        }
    }
};

class VersionedMockSerializationMessage2 : public MessageTemplate<VersionedMockV2, VersionedMockSerializationMessage2>
{
public:
    void serialize(SerializationBuffer& data, SemanticVersion& version) const override
    {
        version = version_;

        MessageTemplate<VersionedMockV2, VersionedMockSerializationMessage2>::serialize(data, version);
    }
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override
    {
        if (version.major_v == 1) {
            // this was a VersionedMockSerializationMessage1

            // read header
            Message::deserialize(data, version);

            // read V1 message (was int instead of string)
            int32_t tmp;
            data >> tmp;
            value.payload = std::to_string(tmp);

        } else if (version.major_v == 123) {
            // special case for testing
            value.payload = "this is version 123";

        } else {
            // default: read from parent
            MessageTemplate<VersionedMockV2, VersionedMockSerializationMessage2>::deserialize(data, version);
        }
        version_ = version;
    }

    SemanticVersion version_;
};

}  // namespace connection_types

class MessageSerializationTest : public CsApexTestCase
{
public:
    MessageSerializationTest()
    {
    }

    virtual ~MessageSerializationTest()
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }
};

namespace connection_types
{
template <>
struct type<VersionedMockSerializationMessage1>
{
    static std::string name()
    {
        return "VersionedMockSerializationMessage1";
    }
};

template <>
struct type<VersionedMockSerializationMessage2>
{
    static std::string name()
    {
        return "VersionedMockSerializationMessage2";
    }
};

}  // namespace connection_types

}  // namespace csapex

namespace YAML
{
template <>
struct convert<csapex::connection_types::VersionedMockSerializationMessage1>
{
    static Node encode(const csapex::connection_types::VersionedMockSerializationMessage1& rhs)
    {
        Node n;
        return n;
    }

    static bool decode(const Node& node, csapex::connection_types::VersionedMockSerializationMessage1& rhs)
    {
        return true;
    }
};
template <>
struct convert<csapex::connection_types::VersionedMockSerializationMessage2>
{
    static Node encode(const csapex::connection_types::VersionedMockSerializationMessage2& rhs)
    {
        Node n;
        return n;
    }

    static bool decode(const Node& node, csapex::connection_types::VersionedMockSerializationMessage2& rhs)
    {
        return true;
    }
};
}  // namespace YAML

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::VersionedMockSerializationMessage1)
CSAPEX_REGISTER_MESSAGE(csapex::connection_types::VersionedMockSerializationMessage2)

namespace csapex
{
TEST_F(MessageSerializationTest, TemplateMessageGetsSemanticVersionFromWrappedClassV1)
{
    connection_types::VersionedMockSerializationMessage1::Ptr message(new connection_types::VersionedMockSerializationMessage1);
    ASSERT_EQ(1, message->getVersion().major_v);
    ASSERT_EQ(0, message->getVersion().minor_v);
    ASSERT_EQ(0, message->getVersion().patch_v);
}

TEST_F(MessageSerializationTest, TemplateMessageGetsSemanticVersionFromWrappedClassV2)
{
    connection_types::VersionedMockSerializationMessage2::Ptr message(new connection_types::VersionedMockSerializationMessage2);
    ASSERT_EQ(2, message->getVersion().major_v);
    ASSERT_EQ(0, message->getVersion().minor_v);
    ASSERT_EQ(0, message->getVersion().patch_v);
}

TEST_F(MessageSerializationTest, LoadAnyVersion)
{
    csapex::SerializationBuffer data;
    {
        connection_types::VersionedMockSerializationMessage2::Ptr message(new connection_types::VersionedMockSerializationMessage2);
        message->version_.major_v = 5;
        message->version_.minor_v = 4;
        message->version_.patch_v = 3;
        message->value.payload = "will not be ignored";
        data << message;
    }
    {
        connection_types::VersionedMockSerializationMessage2::Ptr message;
        data >> message;
        ASSERT_EQ(5, message->version_.major_v);
        ASSERT_EQ(4, message->version_.minor_v);
        ASSERT_EQ(3, message->version_.patch_v);
        ASSERT_STREQ("will not be ignored", message->value.payload.c_str());
    }
}

TEST_F(MessageSerializationTest, LoadSpecificVersion)
{
    csapex::SerializationBuffer data;
    {
        connection_types::VersionedMockSerializationMessage2::Ptr message(new connection_types::VersionedMockSerializationMessage2);
        message->version_.major_v = 123;
        message->version_.minor_v = 0;
        message->version_.patch_v = 0;
        message->value.payload = "will be ignored";
        data << message;
    }
    {
        connection_types::VersionedMockSerializationMessage2::Ptr message;
        data >> message;
        ASSERT_EQ(123, message->version_.major_v);
        ASSERT_EQ(0, message->version_.minor_v);
        ASSERT_EQ(0, message->version_.patch_v);
        ASSERT_STREQ("this is version 123", message->value.payload.c_str());
    }
}

TEST_F(MessageSerializationTest, LoadOldVersion)
{
    csapex::SerializationBuffer data;
    {
        connection_types::VersionedMockSerializationMessage1::Ptr message(new connection_types::VersionedMockSerializationMessage1);
        ASSERT_EQ(1, message->getVersion().major_v);
        ASSERT_EQ(0, message->getVersion().minor_v);
        ASSERT_EQ(0, message->getVersion().patch_v);
        message->value.payload = 123;
        data << message;
    }
    // replace VersionedMockSerializationMessage1 with VersionedMockSerializationMessage2 to fake old message loading
    auto pos = std::find(data.begin(), data.end(), '1');
    *pos = '2';
    {
        connection_types::VersionedMockSerializationMessage2::Ptr message;
        data >> message;
        ASSERT_NE(nullptr, message);
        ASSERT_STREQ("123", message->value.payload.c_str());
    }
}

}  // namespace csapex
