#include <csapex/model/graph_facade.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/execution_type.h>
#include <csapex/model/node_state.h>
#include <csapex/msg/message_allocator.h>
#include <csapex/msg/output.h>

#include <csapex_testing/test_exception_handler.h>
#include <csapex_testing/mockup_nodes.h>
#include <csapex_testing/stepping_test.h>
#include <csapex_testing/io.h>

#include <csapex/msg/generic_vector_message.hpp>

/// SYSTEM
#include <boost/interprocess/managed_shared_memory.hpp>

namespace csapex {


class OutputAllocationTest : public SteppingTest
{
protected:
    NodeFacadeImplementationPtr makeNode(const std::string& type, const UUID& id, GraphPtr graph, ExecutionType exec_type)
    {
        NodeStatePtr state = std::make_shared<NodeState>(nullptr);
        state->setExecutionType(exec_type);
        NodeFacadeImplementationPtr nf = factory.makeNode(type, id, graph, state);

        EXPECT_NE(nullptr, nf);

        return nf;
    }

    void testStepping(ExecutionType exec_type)
    {
        NodeFacadeImplementationPtr src1 = makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src1"), graph, exec_type);
        main_graph_facade->addNode(src1);

        NodeFacadeImplementationPtr src2 = makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src2"), graph, exec_type);
        ASSERT_NE(nullptr, src2);
        main_graph_facade->addNode(src2);


        NodeFacadeImplementationPtr combiner = makeNode("DynamicMultiplier", UUIDProvider::makeUUID_without_parent("combiner"), graph, exec_type);
        ASSERT_NE(nullptr, combiner);
        main_graph_facade->addNode(combiner);

        NodeFacadeImplementationPtr sink_p = makeNode("MockupSink", UUIDProvider::makeUUID_without_parent("Sink"), graph, exec_type);
        main_graph_facade->addNode(sink_p);
        std::shared_ptr<MockupSink> sink = std::dynamic_pointer_cast<MockupSink>(sink_p->getNode());
        ASSERT_NE(nullptr, sink);


        // NESTED GRAPH
        NodeFacadeImplementationPtr sub_graph_node_facade = makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph, exec_type);
        SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
        apex_assert_hard(sub_graph);

        GraphFacadeImplementation sub_graph_facade(executor, sub_graph->getLocalGraph(), sub_graph);

        NodeFacadeImplementationPtr m = makeNode("DynamicMultiplier", UUIDProvider::makeUUID_without_parent("m"), sub_graph->getLocalGraph(), exec_type);
        ASSERT_NE(nullptr, m);
        sub_graph_facade.addNode(m);

        apex_assert_hard(sub_graph_node_facade);
        graph->addNode(sub_graph_node_facade);

        auto type = makeEmpty<connection_types::GenericValueMessage<int> >();

        auto in1_map = sub_graph->addForwardingInput(type, "forwarding", false);
        auto in2_map = sub_graph->addForwardingInput(type, "forwarding", false);
        auto out_map = sub_graph->addForwardingOutput(type, "forwarding");

        // forwarding connections
        sub_graph_facade.connect(in1_map.internal, m, "input_a");
        sub_graph_facade.connect(in2_map.internal, m, "input_b");
        sub_graph_facade.connect(m, "output", out_map.internal);

        // top level connections
        main_graph_facade->connect(combiner, "output", sink_p, "input");

        // crossing connections
        main_graph_facade->connect(src1, "output", in1_map.external);
        main_graph_facade->connect(src2, "output", in2_map.external);
        main_graph_facade->connect(out_map.external, combiner, "input_a");
        main_graph_facade->connect(out_map.external, combiner, "input_b");

        executor.start();

        // execution
        ASSERT_EQ(-1, sink->getValue());
        for(int iter = 0; iter < 100; ++iter) {
            ASSERT_NO_FATAL_FAILURE(step());

            int v = (iter * iter) * (iter * iter);
            ASSERT_EQ(v, sink->getValue());
        }
    }

};

TEST_F(OutputAllocationTest, DefaultMessageAllocator)
{
    MessageAllocator allocator;

    connection_types::GenericValueMessage<int>::Ptr msgptr = allocator.allocate<connection_types::GenericValueMessage<int>>(42, "frame");
    ASSERT_NE(nullptr, msgptr);

    EXPECT_EQ(42, msgptr->value);
    EXPECT_STREQ("frame", msgptr->frame_id.c_str());
}


TEST_F(OutputAllocationTest, OutputCanBeUsedToAllocateMessages)
{
    NodeFacadeImplementationPtr nf = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src1"), graph);
    ASSERT_NE(nullptr, nf);

    OutputPtr output = testing::getOutput(nf, "out_0");
    ASSERT_NE(nullptr, output);

    connection_types::GenericValueMessage<int>::Ptr msgptr = output->template allocate<connection_types::GenericValueMessage<int>>(42, "frame");
    ASSERT_NE(nullptr, msgptr);

    EXPECT_EQ(42, msgptr->value);
    EXPECT_STREQ("frame", msgptr->frame_id.c_str());
}


TEST_F(OutputAllocationTest, OutputCanBeUsedToAllocateMessagesWithForwardDeclaration)
{
    NodeFacadeImplementationPtr nf = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src1"), graph);
    ASSERT_NE(nullptr, nf);

    OutputPtr output = testing::getOutput(nf, "out_0");
    ASSERT_NE(nullptr, output);

    connection_types::GenericValueMessage<int>::Ptr msgptr = msg::allocate<connection_types::GenericValueMessage<int>>(output.get(), 42, "frame");
    ASSERT_NE(nullptr, msgptr);

    EXPECT_EQ(42, msgptr->value);
    EXPECT_STREQ("frame", msgptr->frame_id.c_str());
}

TEST_F(OutputAllocationTest, AllocatorCanBeSet)
{
    NodeFacadeImplementationPtr nf = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src1"), graph);
    ASSERT_NE(nullptr, nf);

    OutputPtr output = testing::getOutput(nf, "out_0");
    ASSERT_NE(nullptr, output);

    using M = connection_types::GenericValueMessage<int>;

    output->setAllocator<M>(std::allocator<uint8_t>());

    M::Ptr msgptr = output->template allocate<M>(42, "frame");
    ASSERT_NE(nullptr, msgptr);

    EXPECT_EQ(42, msgptr->value);
    EXPECT_STREQ("frame", msgptr->frame_id.c_str());
}


using namespace boost::interprocess;

template <typename T>
using ShmAllocator = allocator<T, managed_shared_memory::segment_manager>;

TEST_F(OutputAllocationTest, ShmAllocatorCanBeSet)
{
    NodeFacadeImplementationPtr nf = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src1"), graph);
    ASSERT_NE(nullptr, nf);

    OutputPtr output = testing::getOutput(nf, "out_0");
    ASSERT_NE(nullptr, output);

    using M = connection_types::GenericValueMessage<int>;

    managed_shared_memory segment(open_or_create, "unit_test", 1024*1024);

    struct shm_remove
    {
        shm_remove() { shared_memory_object::remove("unit_test"); }
        ~shm_remove(){ shared_memory_object::remove("unit_test"); }
    } remover;

    try {
        output->setAllocator<M>(ShmAllocator<uint8_t>(segment.get_segment_manager()));

        M::Ptr msgptr = output->template allocate<M>(42, "frame");
        ASSERT_TRUE(segment.belongs_to_segment(msgptr.get()));

        ASSERT_NE(nullptr, msgptr);

        EXPECT_EQ(42, msgptr->value);
        EXPECT_STREQ("frame", msgptr->frame_id.c_str());
    } catch(...) {
        FAIL();
    }
}


TEST_F(OutputAllocationTest, SteppingWorksForProcessingGraphsInSubprocess)
{
    // MAIN GRAPH
    testStepping(ExecutionType::SUBPROCESS);
}

}
