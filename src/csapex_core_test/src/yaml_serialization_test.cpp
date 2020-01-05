#include <csapex_testing/csapex_test_case.h>

#include <csapex/msg/message_template.hpp>
#include <csapex/utility/register_msg.h>
#include <csapex/utility/yaml.h>
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

TEST_F(YAMLSerializationTest, YamlDeserializationWorksWithGenericVector)
{
    for (int i = 0; i < 10; i++) {
        YAML::Node node;

        {
            GenericVectorMessage::Ptr message = GenericVectorMessage::make<MockMessage>();


            std::shared_ptr<std::vector<MockMessage>> vector = std::make_shared<std::vector<MockMessage>>();
            for (int i = 0; i < 10; ++i) {
                vector->emplace_back();
                vector->back().payload = std::to_string(i);
            }
            message->set(vector);

            TokenData::Ptr generic = message;
            node = csapex::MessageSerializer::serializeYamlMessage(*generic);
        }

        {
            TokenData::Ptr generic = csapex::MessageSerializer::deserializeYamlMessage(node);
            ASSERT_NE(nullptr, generic);

            GenericVectorMessage::Ptr vector_msg = std::dynamic_pointer_cast<GenericVectorMessage>(generic);
            ASSERT_NE(nullptr, vector_msg);

            std::shared_ptr<const std::vector<MockMessage>> vector = vector_msg->makeShared<MockMessage>();
            ASSERT_NE(nullptr, vector);

            ASSERT_EQ(10u, vector->size());
        }
    }
}
