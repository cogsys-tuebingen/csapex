#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/node_state.h>
#include <csapex/model/direct_node_worker.h>
#include <csapex/model/subprocess_node_worker.h>
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
class NodeWorkerTest : public SteppingTest
{

};

void runTest(NodeFacadeImplementationPtr node_facade)
{
    Node& node = *node_facade->getNode();
    NodeHandle& nh = *node_facade->getNodeHandle();

    ASSERT_TRUE(node.canProcess());
    ASSERT_FALSE(node_facade->isProcessing());
    ASSERT_FALSE(node_facade->canProcess());

    // no inputs are sent yet -> expect an assertion failure
    ASSERT_THROW(node_facade->startProcessingMessages(), csapex::HardAssertionFailure);

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
    ASSERT_TRUE(node_facade->canProcess());
    ASSERT_TRUE(nh.getInputTransition()->isEnabled());
    ASSERT_TRUE(nh.getOutputTransition()->isEnabled());

    // no inputs are sent now -> node should multiply the input by 4
    ASSERT_FALSE(node_facade->isProcessing());
    ASSERT_TRUE(node_facade->startProcessingMessages());

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


TEST_F(NodeWorkerTest, NodeWorkerProcessing)
{
    NodeFacadeImplementationPtr times_4 = factory.makeNode("StaticMultiplier4",
                                                           UUIDProvider::makeUUID_without_parent("StaticMultiplier4"),
                                                           graph);

    runTest(times_4);
}


TEST_F(NodeWorkerTest, NodeWorkerCanBeSetOnInitialization)
{
    NodeFacadeImplementationPtr direct_times_4 = factory.makeNode("StaticMultiplier4",
                                                           UUIDProvider::makeUUID_without_parent("StaticMultiplier4"),
                                                           graph,
                                                           ExecutionType::DIRECT);
    ASSERT_EQ(ExecutionType::DIRECT, direct_times_4->getNodeState()->getExecutionType());

    NodeFacadeImplementationPtr sp_times_4 = factory.makeNode("StaticMultiplier4",
                                                           UUIDProvider::makeUUID_without_parent("StaticMultiplier4"),
                                                           graph,
                                                           ExecutionType::SUBPROCESS);
    ASSERT_EQ(ExecutionType::SUBPROCESS, sp_times_4->getNodeState()->getExecutionType());
}

TEST_F(NodeWorkerTest, NodeWorkerCanBeSwappedAfterConstruction)
{
    NodeFacadeImplementationPtr times_4 = factory.makeNode("StaticMultiplier4",
                                                           UUIDProvider::makeUUID_without_parent("StaticMultiplier4"),
                                                           graph);

    times_4->replaceNodeWorker(std::make_shared<DirectNodeWorker>(times_4->getNodeHandle()));

    runTest(times_4);
}

TEST_F(NodeWorkerTest, DirectNodeWorkerWorks)
{
    NodeFacadeImplementationPtr times_4 = factory.makeNode("StaticMultiplier4",
                                                           UUIDProvider::makeUUID_without_parent("StaticMultiplier4"),
                                                           graph,
                                                           ExecutionType::DIRECT);

    runTest(times_4);
}


TEST_F(NodeWorkerTest, SubprocessNodeWorkerWorks)
{
    NodeFacadeImplementationPtr times_4 = factory.makeNode("StaticMultiplier4",
                                                           UUIDProvider::makeUUID_without_parent("StaticMultiplier4"),
                                                           graph,
                                                           ExecutionType::SUBPROCESS);

    runTest(times_4);
}

TEST_F(NodeWorkerTest, NodeWorkerCanBeSwappedOnTheFly)
{
    // TODO
}
TEST_F(NodeWorkerTest, SubprocessNodeWorkerCanHandleSegmentationFault)
{
    // TODO
}
TEST_F(NodeWorkerTest, SubprocessNodeWorkerReceivesErrorMessagesFromSubprocess)
{
    // TODO
}
}

