#include <csapex_testing/csapex_test_case.h>

#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex_testing/mockup_msgs.h>

using namespace csapex;
using namespace connection_types;

class CloningTest : public CsApexTestCase
{
protected:
    CloningTest()
    {
    }

    virtual ~CloningTest()
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

TEST_F(CloningTest, VectorClonePerformsDeepCopyMessageImplementation)
{
    const int target_size = 10;
    GenericVectorMessage::Ptr original_message;
    {
        // use message implementation overload
        original_message = GenericVectorMessage::make<MockMessage>();
        for (int i = 0; i < target_size; ++i) {
            MockMessage::Ptr msg(new MockMessage);
            msg->value.payload = std::to_string(i);
            original_message->addNestedValue(msg);
        }
    }

    {
        GenericVectorMessage::Ptr extended_message = msg::message_cast<GenericVectorMessage>(original_message->cloneRaw());
        ASSERT_NE(nullptr, extended_message);

        for (int i = 0; i < target_size; ++i) {
            MockMessage::Ptr msg(new MockMessage);
            msg->value.payload = std::to_string(i);
            extended_message->addNestedValue(msg);
        }

        // the cloned message was extended and should be double the size
        ASSERT_EQ(2 * target_size, extended_message->nestedValueCount());
        // but the original message should still be the same size!
        ASSERT_EQ(target_size, original_message->nestedValueCount());
    }
}

TEST_F(CloningTest, VectorClonePerformsDeepCopyImplementation)
{
    const int target_size = 10;
    GenericVectorMessage::Ptr original_message;
    {
        // use implementation overload
        original_message = GenericVectorMessage::make<std::string>();
        auto shared_vector = std::make_shared<std::vector<std::string>>();
        for (int i = 0; i < target_size; ++i) {
            shared_vector->push_back(std::to_string(i));
        }
        original_message->set(shared_vector);
    }

    {
        GenericVectorMessage::Ptr extended_message = msg::message_cast<GenericVectorMessage>(original_message->cloneRaw());
        ASSERT_NE(nullptr, extended_message);

        for (int i = 0; i < target_size; ++i) {
            GenericValueMessage<std::string>::Ptr msg(new GenericValueMessage<std::string>);
            msg->value = std::to_string(i);
            extended_message->addNestedValue(msg);
        }

        // the cloned message was extended and should be double the size
        ASSERT_EQ(2 * target_size, extended_message->nestedValueCount());
        // but the original message should still be the same size!
        ASSERT_EQ(target_size, original_message->nestedValueCount());
    }
}

TEST_F(CloningTest, VectorClonePerformsDeepCopyInstancedImplementation)
{
    const int target_size = 10;
    GenericVectorMessage::Ptr original_message;
    {
        // use instanced overload
        original_message = GenericVectorMessage::make(std::make_shared<MockMessage>());
        for (int i = 0; i < target_size; ++i) {
            MockMessage::Ptr msg(new MockMessage);
            msg->value.payload = std::to_string(i);
            original_message->addNestedValue(msg);
        }
    }

    {
        GenericVectorMessage::Ptr extended_message = msg::message_cast<GenericVectorMessage>(original_message->cloneRaw());
        ASSERT_NE(nullptr, extended_message);

        for (int i = 0; i < target_size; ++i) {
            MockMessage::Ptr msg(new MockMessage);
            msg->value.payload = std::to_string(i);
            extended_message->addNestedValue(msg);
        }

        // the cloned message was extended and should be double the size
        ASSERT_EQ(2 * target_size, extended_message->nestedValueCount());
        // but the original message should still be the same size!
        ASSERT_EQ(target_size, original_message->nestedValueCount());
    }
}