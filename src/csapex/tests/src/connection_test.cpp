#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/factory/node_factory.h>
#include <csapex/msg/message.h>
#include <csapex/msg/input.h>
#include <csapex/msg/static_output.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/direct_connection.h>
#include <csapex/msg/direct_connection.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/utility/exceptions.h>

#include "gtest/gtest.h"

using namespace csapex;
using namespace connection_types;


class ConnectionTest : public ::testing::Test {
protected:
    ConnectionTest()
        : uuid_provider(std::make_shared<UUIDProvider>())
    {
    }

    virtual ~ConnectionTest() {
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

    UUIDProviderPtr uuid_provider;
};

TEST_F(ConnectionTest, DirectConnectionCompatibility) {
    StaticOutput o(uuid_provider->makeUUID("out"));
    Input i(uuid_provider->makeUUID("in"));

    // non typed should always be connectable!
    ASSERT_TRUE(o.canConnectTo(&i, false));

    o.setType(connection_types::makeEmpty<GenericValueMessage<int>>());
    i.setType(connection_types::makeEmpty<GenericValueMessage<float>>());
    ASSERT_FALSE(o.canConnectTo(&i, false));

    o.setType(connection_types::makeEmpty<GenericValueMessage<int>>());
    i.setType(connection_types::makeEmpty<GenericValueMessage<int>>());
    ASSERT_TRUE(o.canConnectTo(&i, false));
}


TEST_F(ConnectionTest, DirectConnectionIsPossible) {
    OutputPtr o = std::make_shared<StaticOutput>(uuid_provider->makeUUID("out"));
    InputPtr i = std::make_shared<Input>(uuid_provider->makeUUID("in"));

    ConnectionPtr connection = DirectConnection::connect(o, i);

    GenericValueMessage<int>::Ptr msg(new GenericValueMessage<int>);
    msg->value = 42;

    o->addMessage(std::make_shared<Token>(msg));
    o->commitMessages(false);
    o->publish();

    TokenConstPtr raw_message = i->getToken();
    ASSERT_TRUE(raw_message != nullptr);

    GenericValueMessage<int>::ConstPtr value_message = std::dynamic_pointer_cast<GenericValueMessage<int> const>(raw_message->getTokenData());
    ASSERT_TRUE(value_message != nullptr);

    EXPECT_EQ(msg->value, value_message->value);
}


TEST_F(ConnectionTest, DirectConnectionCanBeRepeated) {
    OutputPtr o = std::make_shared<StaticOutput>(uuid_provider->makeUUID("out"));
    InputPtr i = std::make_shared<Input>(uuid_provider->makeUUID("in"));

    ConnectionPtr connection = DirectConnection::connect(o, i);

    auto send = [&](int val) {
        GenericValueMessage<int>::Ptr msg(new GenericValueMessage<int>);
        msg->value = val;

        o->addMessage(std::make_shared<Token>(msg));
        o->commitMessages(false);
        o->publish();
    };

    auto recv = [&](int expected) {
        TokenConstPtr raw_message = i->getToken();
        ASSERT_TRUE(raw_message != nullptr);

        GenericValueMessage<int>::ConstPtr value_message = std::dynamic_pointer_cast<GenericValueMessage<int> const>(raw_message->getTokenData());
        ASSERT_TRUE(value_message != nullptr);

        EXPECT_EQ(expected, value_message->value);
    };

    send(42);
    recv(42);

    send(23);
    recv(23);
}

TEST_F(ConnectionTest, InputsCanBeConnectedToOnlyOneOutput) {
    OutputPtr o1 = std::make_shared<StaticOutput>(uuid_provider->makeUUID("o1"));
    OutputPtr o2 = std::make_shared<StaticOutput>(uuid_provider->makeUUID("o2"));

    InputPtr i = std::make_shared<Input>(uuid_provider->makeUUID("in"));

    DirectConnection::connect(o1, i);

    try {
        DirectConnection::connect(o2, i);
        FAIL();

    } catch(const csapex::Failure& e) {
        SUCCEED();
    }
}


TEST_F(ConnectionTest, DirectConnectionCanBeDeleted) {
    OutputPtr o = std::make_shared<StaticOutput>(uuid_provider->makeUUID("out"));
    InputPtr i = std::make_shared<Input>(uuid_provider->makeUUID("in"));

    ConnectionPtr connection = DirectConnection::connect(o, i);

    GenericValueMessage<int>::Ptr msg(new GenericValueMessage<int>);
    msg->value = 42;

    o->addMessage(std::make_shared<Token>(msg));
    o->commitMessages(false);

    o->removeConnection(i.get());

    o->publish();

    auto raw_message = i->getToken();
    ASSERT_TRUE(raw_message == nullptr);
}


