#include "gtest/gtest.h"

#include <csapex/msg/message_template.hpp>
#include <csapex/utility/register_msg.h>
#include <yaml-cpp/yaml.h>
#include <csapex/serialization/message_serializer.h>
#include <csapex/msg/io.h>

using namespace csapex;
using namespace connection_types;

namespace csapex
{
namespace connection_types
{

class Mock
{
public:
    std::string payload;
};

class MockMessage : public MessageTemplate<Mock, MockMessage> {};

template <>
struct type<MockMessage> {
    static std::string name() {
        return "MockMessage";
    }
};

}
}


/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::MockMessage>
{
    static Node encode(const csapex::connection_types::MockMessage& rhs)
    {
        Node n;
        n["payload"] = rhs.value.payload;
        return n;
    }

    static bool decode(const Node& node, csapex::connection_types::MockMessage& rhs)
    {
        if(node["payload"].IsDefined()) {
            rhs.value.payload = node["payload"].as<std::string>();
        } else {
            return false;
        }
        return true;
    }
};
}

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::MockMessage);

class YAMLSerializationTest : public ::testing::Test
{
protected:
    YAMLSerializationTest()
    {
    }

    virtual ~YAMLSerializationTest() {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp() override {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    virtual void TearDown() override {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

};

TEST_F(YAMLSerializationTest, YamlSerializationWorks)
{
    MockMessage msg;
    msg.value.payload = "foo";

    YAML::Node node = MessageSerializer::serializeMessage(msg);

    ASSERT_TRUE(node["data"].IsDefined());
    ASSERT_TRUE(node["data"]["payload"].IsDefined());
    EXPECT_EQ("foo", node["data"]["payload"].as<std::string>());
}

TEST_F(YAMLSerializationTest, YamlDeserializationWorks)
{
    YAML::Node node;
    node["type"] = "MockMessage";
    node["data"]["payload"] = "bar";

    TokenData::Ptr msg = csapex::MessageSerializer::deserializeMessage(node);
    ASSERT_NE(nullptr, msg);

    MockMessage::Ptr mockmsg = msg::message_cast<MockMessage>(msg);
    ASSERT_NE(nullptr, msg);

    EXPECT_EQ("bar", mockmsg->value.payload);
}
