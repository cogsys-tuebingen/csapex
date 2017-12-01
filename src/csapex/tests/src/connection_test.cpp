#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/msg/message.h>
#include <csapex/msg/input.h>
#include <csapex/msg/static_output.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
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
    ASSERT_TRUE(Connection::isCompatibleWith(&o, &i));

    o.setType(connection_types::makeEmpty<GenericValueMessage<int>>());
    i.setType(connection_types::makeEmpty<GenericValueMessage<float>>());
    ASSERT_FALSE(Connection::isCompatibleWith(&o, &i));

    o.setType(connection_types::makeEmpty<GenericValueMessage<int>>());
    i.setType(connection_types::makeEmpty<GenericValueMessage<int>>());
    ASSERT_TRUE(Connection::isCompatibleWith(&o, &i));
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

    // is compatible
    ASSERT_TRUE(Connection::isCompatibleWith(o1.get(), i.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(i.get(), o1.get()));
    // can be connected
    ASSERT_TRUE(Connection::canBeConnectedTo(o1.get(), i.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(i.get(), o1.get()));

    DirectConnection::connect(o1, i);

    // cannot connect twice
    ASSERT_FALSE(Connection::canBeConnectedTo(o1.get(), i.get()));
    ASSERT_FALSE(Connection::canBeConnectedTo(i.get(), o1.get()));
    // cannot have more than one connection
    ASSERT_FALSE(Connection::canBeConnectedTo(o2.get(), i.get()));
    ASSERT_FALSE(Connection::canBeConnectedTo(i.get(), o2.get()));

    ASSERT_THROW(DirectConnection::connect(o2, i), csapex::Failure);
    ASSERT_THROW(DirectConnection::connect(o1, i), csapex::Failure);
}

//test moving connections as well...

TEST_F(ConnectionTest, OutputsCanHaveManyConnections) {
    OutputPtr o = std::make_shared<StaticOutput>(uuid_provider->makeUUID("o"));

    InputPtr i1 = std::make_shared<Input>(uuid_provider->makeUUID("i1"));
    InputPtr i2 = std::make_shared<Input>(uuid_provider->makeUUID("i2"));

    // is compatible
    ASSERT_TRUE(Connection::isCompatibleWith(o.get(), i1.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(i1.get(), o.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(o.get(), i2.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(i2.get(), o.get()));
    // can be connected
    ASSERT_TRUE(Connection::canBeConnectedTo(o.get(), i1.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(i1.get(), o.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(o.get(), i2.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(i2.get(), o.get()));

    DirectConnection::connect(o, i1);

    // cannot connect twice
    ASSERT_FALSE(Connection::canBeConnectedTo(o.get(), i1.get()));
    ASSERT_FALSE(Connection::canBeConnectedTo(i1.get(), o.get()));

    // can have more than one connection
    ASSERT_TRUE(Connection::canBeConnectedTo(o.get(), i2.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(i2.get(), o.get()));

    DirectConnection::connect(o, i2);

    // cannot connect twice
    ASSERT_FALSE(Connection::canBeConnectedTo(o.get(), i1.get()));
    ASSERT_FALSE(Connection::canBeConnectedTo(i1.get(), o.get()));
    // cannot connect twice
    ASSERT_FALSE(Connection::canBeConnectedTo(o.get(), i2.get()));
    ASSERT_FALSE(Connection::canBeConnectedTo(i2.get(), o.get()));

    // connecting twice throws
    ASSERT_THROW(DirectConnection::connect(o, i1), csapex::Failure);
    ASSERT_THROW(DirectConnection::connect(o, i2), csapex::Failure);
}

TEST_F(ConnectionTest, EventsCanHaveManyConnections) {
    EventPtr e = std::make_shared<Event>(uuid_provider->makeUUID("e"));

    SlotPtr s1 = std::make_shared<Slot>([](const TokenConstPtr& ){}, uuid_provider->makeUUID("s1"), false);
    SlotPtr s2 = std::make_shared<Slot>([](const TokenConstPtr& ){}, uuid_provider->makeUUID("s2"), false);

    // is compatible
    ASSERT_TRUE(Connection::isCompatibleWith(e.get(), s1.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(s1.get(), e.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(e.get(), s2.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(s2.get(), e.get()));
    // can be connected
    ASSERT_TRUE(Connection::canBeConnectedTo(e.get(), s1.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(s1.get(), e.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(e.get(), s2.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(s2.get(), e.get()));

    DirectConnection::connect(e, s1);

    // cannot connect twice
    ASSERT_FALSE(Connection::canBeConnectedTo(e.get(), s1.get()));
    ASSERT_FALSE(Connection::canBeConnectedTo(s1.get(), e.get()));

    // can have more than one connection
    ASSERT_TRUE(Connection::canBeConnectedTo(e.get(), s2.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(s2.get(), e.get()));

    DirectConnection::connect(e, s2);

    // cannot connect twice
    ASSERT_FALSE(Connection::canBeConnectedTo(e.get(), s1.get()));
    ASSERT_FALSE(Connection::canBeConnectedTo(s1.get(), e.get()));
    // cannot connect twice
    ASSERT_FALSE(Connection::canBeConnectedTo(e.get(), s2.get()));
    ASSERT_FALSE(Connection::canBeConnectedTo(s2.get(), e.get()));

    // connecting twice throws
    ASSERT_THROW(DirectConnection::connect(e, s1), csapex::Failure);
    ASSERT_THROW(DirectConnection::connect(e, s2), csapex::Failure);
}

TEST_F(ConnectionTest, SlotsCanHaveManyConnections) {
    EventPtr e1 = std::make_shared<Event>(uuid_provider->makeUUID("e1"));
    EventPtr e2 = std::make_shared<Event>(uuid_provider->makeUUID("e2"));

    SlotPtr s = std::make_shared<Slot>([](const TokenConstPtr& ){}, uuid_provider->makeUUID("s"), false);

    // is compatible
    ASSERT_TRUE(Connection::isCompatibleWith(e1.get(), s.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(s.get(), e1.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(e2.get(), s.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(s.get(), e2.get()));
    // can be connected
    ASSERT_TRUE(Connection::canBeConnectedTo(e1.get(), s.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(s.get(), e1.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(e2.get(), s.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(s.get(), e2.get()));

    DirectConnection::connect(e1, s);

    // cannot connect twice
    ASSERT_FALSE(Connection::canBeConnectedTo(e1.get(), s.get()));
    ASSERT_FALSE(Connection::canBeConnectedTo(s.get(), e1.get()));

    // can have more than one connection
    ASSERT_TRUE(Connection::canBeConnectedTo(e2.get(), s.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(s.get(), e2.get()));

    DirectConnection::connect(e2, s);

    // cannot connect twice
    ASSERT_FALSE(Connection::canBeConnectedTo(e1.get(), s.get()));
    ASSERT_FALSE(Connection::canBeConnectedTo(s.get(), e1.get()));
    // cannot connect twice
    ASSERT_FALSE(Connection::canBeConnectedTo(e2.get(), s.get()));
    ASSERT_FALSE(Connection::canBeConnectedTo(s.get(), e2.get()));

    // connecting twice throws
    ASSERT_THROW(DirectConnection::connect(e1, s), csapex::Failure);
    ASSERT_THROW(DirectConnection::connect(e2, s), csapex::Failure);
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


