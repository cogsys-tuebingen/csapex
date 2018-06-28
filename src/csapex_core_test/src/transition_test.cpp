#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/msg/message.h>
#include <csapex/msg/input.h>
#include <csapex/msg/static_output.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/direct_connection.h>
#include <csapex/msg/direct_connection.h>
#include <csapex/model/connection.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/msg/output_transition.h>
#include <csapex/msg/input_transition.h>

#include <csapex_testing/csapex_test_case.h>

using namespace csapex;
using namespace connection_types;


class TransitionTest : public CsApexTestCase {
protected:
    TransitionTest()
        : uuid_provider(std::make_shared<UUIDProvider>())
    {
    }

    virtual ~TransitionTest() {
    }

    virtual void SetUp() override {
        o1 = std::make_shared<StaticOutput>(uuid_provider->makeUUID("out1"));
        o2 = std::make_shared<StaticOutput>(uuid_provider->makeUUID("out2"));
        o3 = std::make_shared<StaticOutput>(uuid_provider->makeUUID("out3"));

        i1 = std::make_shared<Input>(uuid_provider->makeUUID("in1"));
        i2 = std::make_shared<Input>(uuid_provider->makeUUID("in2"));
        i3 = std::make_shared<Input>(uuid_provider->makeUUID("in3"));
        i4 = std::make_shared<Input>(uuid_provider->makeUUID("in4"));
        i5 = std::make_shared<Input>(uuid_provider->makeUUID("in5"));
    }

    virtual void TearDown() override {
        o1.reset();
        o2.reset();
        o3.reset();

        i1.reset();
        i2.reset();
        i3.reset();
        i4.reset();
        i5.reset();
    }

protected:
    OutputPtr o1;
    OutputPtr o2;
    OutputPtr o3;

    InputPtr i1;
    InputPtr i2;
    InputPtr i3;
    InputPtr i4;
    InputPtr i5;

    UUIDProviderPtr uuid_provider;

protected:
    void ASSERT_RECEIVED (Input& i, int expected) {
        auto raw_message = i.getToken();
        ASSERT_TRUE(raw_message != nullptr);

        GenericValueMessage<int>::ConstPtr value_message = std::dynamic_pointer_cast<GenericValueMessage<int> const>(raw_message->getTokenData());
        ASSERT_TRUE(value_message != nullptr);

        ASSERT_EQ(value_message->value, expected);
    }


    void ASSERT_IN_CONNECTION(const Connection& c, int expected) {
        auto raw_message = c.getToken();
        ASSERT_TRUE(raw_message != nullptr);

        GenericValueMessage<int>::ConstPtr value_message = std::dynamic_pointer_cast<GenericValueMessage<int> const>(raw_message->getTokenData());
        ASSERT_TRUE(value_message != nullptr);

        ASSERT_EQ(value_message->value, expected);
    }

    void sendMessage (Output& o, int i) {
        GenericValueMessage<int>::Ptr msg(new GenericValueMessage<int>);
        msg->value = i;
        o.addMessage(std::make_shared<Token>(msg));
    }
};

TEST_F(TransitionTest, TestOutputTransitionSequenceIds)
{
    OutputTransition ot;
    ot.addOutput(o1);
    ot.addOutput(o2);

    auto ASSERT_SEQ_NO = [&](int i) {
        ASSERT_EQ(i, ot.getSequenceNumber());
        ASSERT_EQ(i, o1->sequenceNumber());
        ASSERT_EQ(i, o2->sequenceNumber());
    };

    // initial
    ASSERT_SEQ_NO(-1);

    sendMessage(*o1,1);
    sendMessage(*o2,1);

    ot.sendMessages(false);

    ASSERT_SEQ_NO(0);

    sendMessage(*o1,2);
    sendMessage(*o2,2);

    ot.sendMessages(false);

    ASSERT_SEQ_NO(1);


    sendMessage(*o1,3);
    sendMessage(*o2,3);

    // set all outputs sequence number
    ot.setSequenceNumber(41);

    ot.sendMessages(false);

    ASSERT_SEQ_NO(42);
}


TEST_F(TransitionTest, TestOutputTransitionSequenceIdsConnected)
{
    OutputTransition ot;
    ot.addOutput(o1);
    ot.addOutput(o2);

    ConnectionPtr c_1a_1 = DirectConnection::connect(o1, i1);
    ConnectionPtr c_1b_2 = DirectConnection::connect(o2, i2);

    auto ASSERT_SEQ_NO = [&](int i) {
        ASSERT_EQ(i, i1->sequenceNumber());
        ASSERT_EQ(i, i2->sequenceNumber());
        ASSERT_EQ(i, ot.getSequenceNumber());
        ASSERT_EQ(i, o1->sequenceNumber());
        ASSERT_EQ(i, o2->sequenceNumber());
    };

    // initial
    ASSERT_SEQ_NO(-1);

    sendMessage(*o1,1);
    sendMessage(*o2,1);

    ot.sendMessages(false);

    ASSERT_SEQ_NO(0);

    sendMessage(*o1,2);
    sendMessage(*o2,2);

    ot.sendMessages(false);

    ASSERT_SEQ_NO(1);


    sendMessage(*o1,3);
    sendMessage(*o2,3);

    // set all outputs seq no
    ot.setSequenceNumber(41);

    ot.sendMessages(false);

    ASSERT_SEQ_NO(42);
}

TEST_F(TransitionTest, TestOutputTransition)
{
    OutputTransition ot;
    ot.addOutput(o1);
    ot.addOutput(o2);
    ot.addOutput(o3);

    /*
     * Outputs are governed by an output transition
     *
     *        +-----I_1
     *        |
     *        |
     * O_1 ---+---I_2
     *        |
     * O_2 ---n---I_3
     *        |
     *        |
     *        +-----I_4
     *
     * O_3 ---------I_5
     */

    ConnectionPtr c_1a_1 = DirectConnection::connect(o1, i1);
    ConnectionPtr c_1a_2 = DirectConnection::connect(o1, i2);
    ConnectionPtr c_1a_4 = DirectConnection::connect(o1, i4);

    ConnectionPtr c_1b_3 = DirectConnection::connect(o2, i3);
    ConnectionPtr c_1c_5 = DirectConnection::connect(o3, i5);

    for(int iter = 0; iter < 4; ++iter) {
        sendMessage(*o1,1);
        sendMessage(*o2,2);
        sendMessage(*o3,3);

        ot.sendMessages(false);

        ASSERT_IN_CONNECTION(*c_1a_1, 1);
        ASSERT_IN_CONNECTION(*c_1a_2, 1);
        ASSERT_IN_CONNECTION(*c_1a_4, 1);
        ASSERT_IN_CONNECTION(*c_1b_3, 2);
        ASSERT_IN_CONNECTION(*c_1c_5, 3);

        ASSERT_RECEIVED(*i1, 1);
        ASSERT_RECEIVED(*i2, 1);
        ASSERT_RECEIVED(*i3, 2);
        ASSERT_RECEIVED(*i4, 1);
        ASSERT_RECEIVED(*i5, 3);
    }
}

TEST_F(TransitionTest, TestInputTransition)
{
    InputTransition it;

    it.addInput(i1);
    it.addInput(i2);
    it.addInput(i3);
    it.addInput(i4);
    it.addInput(i5);

    /*
     * Inputs are governed by an input transition
     *
     *        +-----I_1
     *        |
     *        |
     * O_1 ---+---I_2
     *        |
     * O_2 ---n---I_3
     *        |
     *        |
     *        +-----I_4
     *
     * O_3 ---------I_5
     */

    ConnectionPtr c_1_1a = DirectConnection::connect(o1, i1);
    ConnectionPtr c_1_1b = DirectConnection::connect(o1, i2);
    ConnectionPtr c_1_1d = DirectConnection::connect(o1, i4);

    ConnectionPtr c_2_1c = DirectConnection::connect(o2, i3);
    ConnectionPtr c_3_1e = DirectConnection::connect(o3, i5);

    // this is necessary, if we don't use output transitions
    auto publish = [](Output& o) {
        o.commitMessages(false);
        o.publish();
    };


    for(int iter = 0; iter < 4; ++iter) {
        // send the first two messages -> should not be enabled
        ASSERT_FALSE(it.isEnabled());
        sendMessage(*o1,1);
        publish(*o1);
        ASSERT_FALSE(it.isEnabled());

        sendMessage(*o2,2);
        publish(*o2);
        ASSERT_FALSE(it.isEnabled());

        // send the final message -> should be enabled
        sendMessage(*o3,3);
        publish(*o3);
        ASSERT_TRUE(it.isEnabled());

        it.forwardMessages();

        ASSERT_RECEIVED(*i1, 1);
        ASSERT_RECEIVED(*i2, 1);
        ASSERT_RECEIVED(*i3, 2);
        ASSERT_RECEIVED(*i4, 1);
        ASSERT_RECEIVED(*i5, 3);

        it.notifyMessageRead();
        it.notifyMessageProcessed();
    }
}

TEST_F(TransitionTest, TestFullTransitions)
{
    OutputTransition ot;

    ot.addOutput(o1);
    ot.addOutput(o2);
    ot.addOutput(o3);

    InputTransition it;

    it.addInput(i1);
    it.addInput(i2);
    it.addInput(i3);
    it.addInput(i4);
    it.addInput(i5);

    /*
     * Both outputs and inputs are governed by transitions
     *
     *        +-----I_1
     *        |
     *        |
     * O_1 ---+---I_2
     *        |
     * O_2 ---n---I_3
     *        |
     *        |
     *        +-----I_4
     *
     * O_3 ---------I_5
     */

    ConnectionPtr c_1a_1a = DirectConnection::connect(o1, i1);
    ConnectionPtr c_1a_1b = DirectConnection::connect(o1, i2);
    ConnectionPtr c_1a_1d = DirectConnection::connect(o1, i4);

    ConnectionPtr c_1b_1c = DirectConnection::connect(o2, i3);
    ConnectionPtr c_1c_1e = DirectConnection::connect(o3, i5);

    for(int iter = 0; iter < 4; ++iter) {
        // send the first two messages -> should not be enabled
        sendMessage(*o1,1);

        sendMessage(*o2,2);

        // send the final message -> should be enabled
        sendMessage(*o3,3);

        ot.sendMessages(false);


        ASSERT_IN_CONNECTION(*c_1a_1a, 1);
        ASSERT_IN_CONNECTION(*c_1a_1b, 1);
        ASSERT_IN_CONNECTION(*c_1a_1d, 1);
        ASSERT_IN_CONNECTION(*c_1b_1c, 2);
        ASSERT_IN_CONNECTION(*c_1c_1e, 3);


        ASSERT_TRUE(it.isEnabled());
        it.forwardMessages();

        ASSERT_RECEIVED(*i1, 1);
        ASSERT_RECEIVED(*i2, 1);
        ASSERT_RECEIVED(*i3, 2);
        ASSERT_RECEIVED(*i4, 1);
        ASSERT_RECEIVED(*i5, 3);

        it.notifyMessageRead();
        it.notifyMessageProcessed();
    }
}

TEST_F(TransitionTest, TestBundledAndDirectConnectionsCanBeMixed)
{
    OutputTransition ot;
    ot.addOutput(o1);

    /*
     * Outputs are governed by an output transition
     *
     *        +-----I_1
     *        |
     *        |
     * O_1 ---+---I_2
     *        |
     * O_2 ---n---I_3
     *        |
     *        |
     *        +-----I_4
     *
     * O_3 ---------I_5
     */

    ConnectionPtr c_1a_1 = DirectConnection::connect(o1, i1);
    ConnectionPtr c_1a_2 = DirectConnection::connect(o1, i2);

    for(int iter = 0; iter < 4; ++iter) {
        sendMessage(*o1,iter);

        ot.sendMessages(false);

        ASSERT_IN_CONNECTION(*c_1a_1, iter);
        ASSERT_IN_CONNECTION(*c_1a_2, iter);

        ASSERT_RECEIVED(*i1, iter);
        ASSERT_RECEIVED(*i2, iter);
    }
}
