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
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/msg/input.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/io.h>
#include <csapex/msg/output_transition.h>
#include <csapex/msg/static_output.h>
#include <csapex/utility/uuid_provider.h>

#include "gtest/gtest.h"
#include <csapex_testing/mockup_nodes.h>
#include <csapex_testing/test_exception_handler.h>

#include <mutex>
#include <condition_variable>

#include <csapex_testing/stepping_test.h>

namespace csapex
{
class NodeWorkerTest : public SteppingTest
{

};

void runSyncTest(NodeFacadeImplementationPtr node_facade, int value = 23)
{
    Node& node = *node_facade->getNode();
    NodeHandle& nh = *node_facade->getNodeHandle();
    NodeWorkerPtr nw = node_facade->getNodeWorker().lock();
    ASSERT_NE(nullptr, nw);

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
    msg::publish(tmp_out.get(), value);

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

    ASSERT_EQ(value * 4, msg_out->value);

    input->removeConnection(tmp_out.get());
}


TEST_F(NodeWorkerTest, NodeWorkerProcessing)
{
    NodeFacadeImplementationPtr times_4 = factory.makeNode("StaticMultiplier4",
                                                           UUIDProvider::makeUUID_without_parent("StaticMultiplier4"),
                                                           graph);

    runSyncTest(times_4, 23);
    runSyncTest(times_4, 42);
}


TEST_F(NodeWorkerTest, NodeWorkerCanBeSetOnInitialization)
{
    {
        NodeStatePtr state = std::make_shared<NodeState>(nullptr);
        state->setExecutionType(ExecutionType::DIRECT);
        NodeFacadeImplementationPtr direct_times_4 = factory.makeNode("StaticMultiplier4",
                                                                      UUIDProvider::makeUUID_without_parent("StaticMultiplier4"),
                                                                      graph,
                                                                      state);
        ASSERT_EQ(ExecutionType::DIRECT, direct_times_4->getNodeState()->getExecutionType());
    }
    {
        NodeStatePtr state = std::make_shared<NodeState>(nullptr);
        state->setExecutionType(ExecutionType::SUBPROCESS);
        NodeFacadeImplementationPtr sp_times_4 = factory.makeNode("StaticMultiplier4",
                                                                  UUIDProvider::makeUUID_without_parent("StaticMultiplier4"),
                                                                  graph,
                                                                  state);
        ASSERT_EQ(ExecutionType::SUBPROCESS, sp_times_4->getNodeState()->getExecutionType());
    }
}

TEST_F(NodeWorkerTest, NodeWorkerCanBeSwappedAfterConstruction)
{
    NodeFacadeImplementationPtr times_4 = factory.makeNode("StaticMultiplier4",
                                                           UUIDProvider::makeUUID_without_parent("StaticMultiplier4"),
                                                           graph);

    times_4->replaceNodeWorker(std::make_shared<DirectNodeWorker>(times_4->getNodeHandle()));

    runSyncTest(times_4, 23);
    runSyncTest(times_4, 42);
}

TEST_F(NodeWorkerTest, DirectNodeWorkerWorks)
{
    NodeStatePtr state = std::make_shared<NodeState>(nullptr);
    state->setExecutionType(ExecutionType::DIRECT);
    NodeFacadeImplementationPtr times_4 = factory.makeNode("StaticMultiplier4",
                                                           UUIDProvider::makeUUID_without_parent("StaticMultiplier4"),
                                                           graph,
                                                           state);

    runSyncTest(times_4, 23);
    runSyncTest(times_4, 42);
}


TEST_F(NodeWorkerTest, SubprocessNodeWorkerWorks)
{
    NodeStatePtr state = std::make_shared<NodeState>(nullptr);
    state->setExecutionType(ExecutionType::SUBPROCESS);
    NodeFacadeImplementationPtr times_4 = factory.makeNode("StaticMultiplier4",
                                                           UUIDProvider::makeUUID_without_parent("StaticMultiplier4"),
                                                           graph,
                                                           state);

    runSyncTest(times_4, 23);
    runSyncTest(times_4, 42);
}

TEST_F(NodeWorkerTest, NodeWorkerCanBeSwappedOnTheFly)
{
    NodeFacadeImplementationPtr times_4 = factory.makeNode("StaticMultiplier4",
                                                           UUIDProvider::makeUUID_without_parent("StaticMultiplier4"),
                                                           graph);

    runSyncTest(times_4, 23);

    times_4->replaceNodeWorker(std::make_shared<SubprocessNodeWorker>(times_4->getNodeHandle()));

    runSyncTest(times_4, 42);
}

TEST_F(NodeWorkerTest, ChangingNodeWorkerDoesNotChangePorts)
{
    NodeFacadeImplementationPtr times_4 = factory.makeNode("StaticMultiplier4",
                                                           UUIDProvider::makeUUID_without_parent("StaticMultiplier4"),
                                                           graph);

    int inputs = times_4->getInputs().size();
    int outputs = times_4->getOutputs().size();
    int events = times_4->getEvents().size();
    int slots_ = times_4->getSlots().size();

    ASSERT_EQ(inputs , times_4->getNodeHandle()->getExternalInputs().size());
    ASSERT_EQ(outputs, times_4->getNodeHandle()->getExternalOutputs().size());
    ASSERT_EQ(events , times_4->getNodeHandle()->getExternalEvents().size());
    ASSERT_EQ(slots_ , times_4->getNodeHandle()->getExternalSlots().size());

    times_4->replaceNodeWorker(std::make_shared<SubprocessNodeWorker>(times_4->getNodeHandle()));

    ASSERT_EQ(inputs , times_4->getNodeHandle()->getExternalInputs().size());
    ASSERT_EQ(outputs, times_4->getNodeHandle()->getExternalOutputs().size());
    ASSERT_EQ(events , times_4->getNodeHandle()->getExternalEvents().size());
    ASSERT_EQ(slots_ , times_4->getNodeHandle()->getExternalSlots().size());

    ASSERT_EQ(inputs , times_4->getInputs().size());
    ASSERT_EQ(outputs, times_4->getOutputs().size());
    ASSERT_EQ(events , times_4->getEvents().size());
    ASSERT_EQ(slots_ , times_4->getSlots().size());
}

TEST_F(NodeWorkerTest, SubprocessNodeWorkerCanHandleSegmentationFault)
{
    // TODO
}

TEST_F(NodeWorkerTest, SubprocessNodeWorkerReceivesErrorMessagesFromSubprocess)
{
    // TODO
}

TEST_F(NodeWorkerTest, SubprocessHandlesSynchronousPorts)
{
    NodeFacadeImplementationPtr node_facade = factory.makeNode("AsyncStaticMultiplier4",
                                                               UUIDProvider::makeUUID_without_parent("AsyncStaticMultiplier4"),
                                                               graph);

    NodeHandle& nh = *node_facade->getNodeHandle();
    NodeWorkerPtr nw = node_facade->getNodeWorker().lock();
    ASSERT_NE(nullptr, nw);

    ASSERT_FALSE(node_facade->isProcessing());

    // create a temporary output and connect it to the input
    OutputPtr tmp_out = std::make_shared<StaticOutput>(UUIDProvider::makeUUID_without_parent("tmp_out"));

    // the first three slots are auto generated
    SlotPtr slot = nh.getSlot(UUIDProvider::makeUUID_without_parent("AsyncStaticMultiplier4:|:slot_2"));
    ASSERT_NE(nullptr, slot);
    ConnectionPtr connection = DirectConnection::connect(tmp_out, slot);

    // set the input message
    msg::publish(tmp_out.get(), 23);

    // mark the messages as committed
    tmp_out->commitMessages(false);
    tmp_out->publish();

    nw->startProcessingSlot(slot);

    // commit the messages produced by the node
    EventPtr event = nh.getEvent(UUIDProvider::makeUUID_without_parent("AsyncStaticMultiplier4:|:event_3"));
    ASSERT_NE(nullptr, event);

    event->commitMessages(false);

    // view outputs
    TokenPtr token_out = event->getToken();
    ASSERT_NE(nullptr, token_out);
    TokenDataConstPtr data_out = token_out->getTokenData();
    ASSERT_NE(nullptr, data_out);

    event->notifyMessageProcessed();

    auto msg_out = std::dynamic_pointer_cast<connection_types::GenericValueMessage<int> const>(data_out);
    ASSERT_NE(nullptr, msg_out);

    ASSERT_EQ(23 * 4, msg_out->value);

    slot->removeConnection(tmp_out.get());
}

TEST_F(NodeWorkerTest, SubprocessHandlesAsynchronousPorts)
{
    NodeFacadeImplementationPtr node_facade = factory.makeNode("AsyncStaticMultiplier4",
                                                               UUIDProvider::makeUUID_without_parent("AsyncStaticMultiplier4"),
                                                               graph);

    node_facade->replaceNodeWorker(std::make_shared<SubprocessNodeWorker>(node_facade->getNodeHandle()));

    NodeHandle& nh = *node_facade->getNodeHandle();
    NodeWorkerPtr nw = node_facade->getNodeWorker().lock();
    ASSERT_NE(nullptr, nw);

    ASSERT_FALSE(node_facade->isProcessing());

    // create a temporary output and connect it to the input
    OutputPtr tmp_out = std::make_shared<StaticOutput>(UUIDProvider::makeUUID_without_parent("tmp_out"));

    // the first three slots are auto generated
    SlotPtr slot = nh.getSlot(UUIDProvider::makeUUID_without_parent("AsyncStaticMultiplier4:|:slot_2"));
    ASSERT_NE(nullptr, slot);
    ConnectionPtr connection = DirectConnection::connect(tmp_out, slot);

    // set the input message
    msg::publish(tmp_out.get(), 23);

    // mark the messages as committed
    tmp_out->commitMessages(false);
    tmp_out->publish();

    nw->startProcessingSlot(slot);

    // commit the messages produced by the node
    EventPtr event = nh.getEvent(UUIDProvider::makeUUID_without_parent("AsyncStaticMultiplier4:|:event_3"));
    ASSERT_NE(nullptr, event);

    event->commitMessages(false);

    // view outputs
    TokenPtr token_out = event->getToken();
    ASSERT_NE(nullptr, token_out);
    TokenDataConstPtr data_out = token_out->getTokenData();
    ASSERT_NE(nullptr, data_out);

    event->notifyMessageProcessed();

    auto msg_out = std::dynamic_pointer_cast<connection_types::GenericValueMessage<int> const>(data_out);
    ASSERT_NE(nullptr, msg_out);

    ASSERT_EQ(23 * 4, msg_out->value);

    slot->removeConnection(tmp_out.get());
}
}

