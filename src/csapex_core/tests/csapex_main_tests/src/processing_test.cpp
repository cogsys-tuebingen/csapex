#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/node_worker.h>
#include <csapex/msg/direct_connection.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/input.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/io.h>
#include <csapex/msg/output_transition.h>
#include <csapex/msg/static_output.h>
#include <csapex/utility/uuid_provider.h>

#include "gtest/gtest.h"
#include <csapex/test/mockup_nodes.h>
#include <csapex/test/test_exception_handler.h>

#include <mutex>
#include <condition_variable>

#include <csapex/test/stepping_test.h>

namespace csapex
{
class ProcessingTest : public NodeConstructingTest
{

};

TEST_F(ProcessingTest, DirectCallToProcess)
{
    NodeFacadeImplementationPtr times_4 = factory.makeNode("StaticMultiplier4", UUIDProvider::makeUUID_without_parent("StaticMultiplier4"), graph);
    Node& node = *times_4->getNode();
    NodeHandle& nh = *times_4->getNodeHandle();

    // no inputs are sent yet -> expect an assertion failure
    ASSERT_TRUE(node.canProcess());
    ASSERT_THROW(node.process(nh, node), csapex::HardAssertionFailure);

    // set the input message
    TokenPtr token_in = msg::createToken(23);
    InputPtr input = nh.getInput(UUIDProvider::makeUUID_without_parent("StaticMultiplier4:|:in_0"));
    ASSERT_NE(nullptr, input);
    input->setToken(token_in);


    // no inputs are sent now -> node should multiply the input by 4
    ASSERT_TRUE(node.canProcess());
    node.process(nh, node);

    // commit the messages produced by the node
    OutputPtr output = nh.getOutput(UUIDProvider::makeUUID_without_parent("StaticMultiplier4:|:out_0"));
    ASSERT_NE(nullptr, output);
    output->commitMessages(false);

    // view outputs
    TokenPtr token_out = output->getToken();
    TokenDataConstPtr data_out = token_out->getTokenData();
    ASSERT_NE(nullptr, data_out);

    auto msg_out = std::dynamic_pointer_cast<connection_types::GenericValueMessage<int> const>(data_out);
    ASSERT_NE(nullptr, msg_out);

    ASSERT_EQ(23 * 4, msg_out->value);
}

TEST_F(ProcessingTest, CallToProcessViaNodeWorker)
{
    NodeFacadeImplementationPtr times_4 = factory.makeNode("StaticMultiplier4", UUIDProvider::makeUUID_without_parent("StaticMultiplier4"), graph);
    Node& node = *times_4->getNode();
    NodeHandle& nh = *times_4->getNodeHandle();

    // no inputs are sent yet -> expect an assertion failure
    ASSERT_TRUE(node.canProcess());
    ASSERT_FALSE(times_4->isProcessing());
    ASSERT_FALSE(times_4->canProcess());
    ASSERT_THROW(times_4->startProcessingMessages(), csapex::HardAssertionFailure);

    // create a temporary output and connect it to the input
    OutputPtr tmp_out = std::make_shared<StaticOutput>(UUIDProvider::makeUUID_without_parent("tmp_out"));
    InputPtr input = nh.getInput(UUIDProvider::makeUUID_without_parent("StaticMultiplier4:|:in_0"));
    ASSERT_NE(nullptr, input);
    ConnectionPtr connection = DirectConnection::connect(tmp_out, input);

    // set the input message
    msg::publish(tmp_out.get(), 23);

    // mark the messages as committed
    tmp_out->commitMessages(false);
    tmp_out->publish();

    ASSERT_TRUE(node.canProcess());
    ASSERT_TRUE(times_4->canProcess());
    ASSERT_TRUE(nh.getInputTransition()->isEnabled());
    ASSERT_TRUE(nh.getOutputTransition()->isEnabled());

    // no inputs are sent now -> node should multiply the input by 4
    ASSERT_FALSE(times_4->isProcessing());
    ASSERT_TRUE(times_4->startProcessingMessages());

    // commit the messages produced by the node
    OutputPtr output = nh.getOutput(UUIDProvider::makeUUID_without_parent("StaticMultiplier4:|:out_0"));
    ASSERT_NE(nullptr, output);

    // view outputs
    TokenPtr token_out = output->getToken();
    TokenDataConstPtr data_out = token_out->getTokenData();
    ASSERT_NE(nullptr, data_out);

    auto msg_out = std::dynamic_pointer_cast<connection_types::GenericValueMessage<int> const>(data_out);
    ASSERT_NE(nullptr, msg_out);

    ASSERT_EQ(23 * 4, msg_out->value);
}
}

