#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/msg/message.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/event.h>
#include <csapex/msg/input.h>
#include <csapex/msg/static_output.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/model/connection.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/msg/direct_connection.h>
#include <csapex/msg/direct_connection.h>

#include "gtest/gtest.h"

using namespace csapex;
using namespace connection_types;


class SignalTest : public ::testing::Test {
protected:
    SignalTest()
        : uuid_provider(std::make_shared<UUIDProvider>())
    {
    }

    virtual ~SignalTest() {
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

TEST_F(SignalTest, DirectConnectionCompatibility) {
    bool test = false;

    EventPtr e = std::make_shared<Event>(uuid_provider->makeUUID("out"));
    SlotPtr s = std::make_shared<Slot>([&](){
        test = true;
    }, uuid_provider->makeUUID("in"), true);

    ConnectionPtr c = DirectConnection::connect(e, s);

    s->triggered.connect([&]() { s->handleEvent(); });  // needed because there is no scheduler involved

    e->trigger();
    e->commitMessages(false);
    e->publish();

    ASSERT_TRUE(test);
}


TEST_F(SignalTest, TypedSignals) {
    int test = -1;

    EventPtr e = std::make_shared<Event>(uuid_provider->makeUUID("out"));
    std::function<void(const csapex::connection_types::GenericValueMessage<int>::ConstPtr&)> fn = [&](const csapex::connection_types::GenericValueMessage<int>::ConstPtr& token){
        test = token->value;
    };
    SlotPtr s = std::make_shared<Slot>(fn, uuid_provider->makeUUID("in"), true);

    ConnectionPtr c = DirectConnection::connect(e, s);

    s->triggered.connect([&]() {s->handleEvent(); }); // needed because there is no scheduler involved

    csapex::connection_types::GenericValueMessage<int>::Ptr token_data(new csapex::connection_types::GenericValueMessage<int>);
    token_data->value = 42;
    e->triggerWith(std::make_shared<Token>(token_data));
    e->commitMessages(false);
    e->publish();
    ASSERT_EQ(42, test);

    token_data->value = 23;
    e->triggerWith(std::make_shared<Token>(token_data));
    e->commitMessages(false);
    e->publish();
    ASSERT_EQ(23, test);
}

TEST_F(SignalTest, SlotsCanBeConnectedToManyOutputs) {
    int test = -1;

    EventPtr e1 = std::make_shared<Event>(uuid_provider->makeUUID("e1"));
    EventPtr e2 = std::make_shared<Event>(uuid_provider->makeUUID("e2"));

    std::function<void(const csapex::connection_types::GenericValueMessage<int>::ConstPtr&)> fn = [&](const csapex::connection_types::GenericValueMessage<int>::ConstPtr& token){
        test = token->value;
    };
    SlotPtr s = std::make_shared<Slot>(fn, uuid_provider->makeUUID("in"), true);

    DirectConnection::connect(e1, s);
    DirectConnection::connect(e2, s);

    s->triggered.connect([&]() {s->handleEvent(); }); // needed because there is no scheduler involved

    csapex::connection_types::GenericValueMessage<int>::Ptr token_data(new csapex::connection_types::GenericValueMessage<int>);
    token_data->value = 42;
    e1->triggerWith(std::make_shared<Token>(token_data));
    e1->commitMessages(false);
    e1->publish();
    ASSERT_EQ(42, test);

    token_data->value = 23;
    e2->triggerWith(std::make_shared<Token>(token_data));
    e2->commitMessages(false);
    e2->publish();
    ASSERT_EQ(23, test);
}


TEST_F(SignalTest, MixingEventsAndInputs) {
    int test = -1;

    EventPtr e = std::make_shared<Event>(uuid_provider->makeUUID("out"));
    std::function<void(const csapex::connection_types::GenericValueMessage<int>::ConstPtr&)> fn = [&](const csapex::connection_types::GenericValueMessage<int>::ConstPtr& token){
        test = token->value;
    };
    InputPtr i = std::make_shared<Input>(uuid_provider->makeUUID("in"));

    DirectConnection::connect(e, i);

    csapex::connection_types::GenericValueMessage<int>::Ptr token_data(new csapex::connection_types::GenericValueMessage<int>);
    token_data->value = 42;

    e->triggerWith(std::make_shared<Token>(token_data));
    e->commitMessages(false);
    e->publish();


    TokenConstPtr raw_message = i->getToken();
    ASSERT_TRUE(raw_message != nullptr);

    GenericValueMessage<int>::ConstPtr value_message = std::dynamic_pointer_cast<GenericValueMessage<int> const>(raw_message->getTokenData());
    ASSERT_TRUE(value_message != nullptr);

    EXPECT_EQ(42, value_message->value);
}

TEST_F(SignalTest, MixingOutputsAndSlots) {
    int test = -1;

    OutputPtr o = std::make_shared<StaticOutput>(uuid_provider->makeUUID("out"));
    std::function<void(const csapex::connection_types::GenericValueMessage<int>::ConstPtr&)> fn = [&](const csapex::connection_types::GenericValueMessage<int>::ConstPtr& token){
        test = token->value;
    };
    SlotPtr s = std::make_shared<Slot>(fn, uuid_provider->makeUUID("in"), true);

    ConnectionPtr c = DirectConnection::connect(o, s);

    s->triggered.connect([&]() {s->handleEvent(); }); // needed because there is no scheduler involved


    GenericValueMessage<int>::Ptr token_data(new GenericValueMessage<int>);
    token_data->value = 42;

    o->addMessage(std::make_shared<Token>(token_data));
    o->commitMessages(false);
    o->publish();

    ASSERT_EQ(42, test);

    token_data->value = 23;

    o->addMessage(std::make_shared<Token>(token_data));
    o->commitMessages(false);
    o->publish();

    ASSERT_EQ(23, test);
}
