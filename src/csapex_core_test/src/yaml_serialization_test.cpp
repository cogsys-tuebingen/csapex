#include <csapex_testing/csapex_test_case.h>

#include <csapex/msg/message_template.hpp>
#include <csapex/utility/register_msg.h>
#include <yaml-cpp/yaml.h>
#include <csapex/serialization/message_serializer.h>
#include <csapex/msg/io.h>
#include <csapex_testing/mockup_msgs.h>

using namespace csapex;
using namespace connection_types;

class YAMLSerializationTest : public CsApexTestCase
{
protected:
    YAMLSerializationTest()
    {
    }

    virtual ~YAMLSerializationTest()
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp() override
    {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    virtual void TearDown() override
    {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }
};

TEST_F(YAMLSerializationTest, YamlSerializationWorks)
{
    MockMessage msg;
    msg.value.payload = "foo";

    YAML::Node node = MessageSerializer::serializeYamlMessage(msg);

    ASSERT_TRUE(node["data"].IsDefined());
    ASSERT_TRUE(node["data"]["payload"].IsDefined());
    ASSERT_EQ("foo", node["data"]["payload"].as<std::string>());
}

TEST_F(YAMLSerializationTest, YamlDeserializationWorks)
{
    YAML::Node node;
    node["type"] = "MockMessage";
    node["data"]["payload"] = "bar";

    TokenData::Ptr msg = csapex::MessageSerializer::deserializeYamlMessage(node);
    ASSERT_NE(nullptr, msg);

    MockMessage::Ptr mockmsg = msg::message_cast<MockMessage>(msg);
    ASSERT_NE(nullptr, msg);

    ASSERT_EQ("bar", mockmsg->value.payload);
}
