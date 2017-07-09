#include <csapex/model/graph_facade.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/model/graph/graph_local.h>

#include "test_exception_handler.h"
#include "mockup_nodes.h"
#include "node_constructing_test.h"

namespace csapex {


class SchedulingTest : public NodeConstructingTest
{

};

TEST_F(SchedulingTest, SteppingWorksForProcessingGraphs) {
    GraphFacade main_graph_facade(executor, graph, graph_node);

    executor.setSteppingMode(true);

    // MAIN GRAPH
    NodeFacadePtr src1 = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src1"), graph);
    ASSERT_NE(nullptr, src1);
    main_graph_facade.addNode(src1);

    NodeFacadePtr src2 = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src2"), graph);
    ASSERT_NE(nullptr, src2);
    main_graph_facade.addNode(src2);


    NodeFacadePtr combiner = factory.makeNode("DynamicMultiplier", UUIDProvider::makeUUID_without_parent("combiner"), graph);
    ASSERT_NE(nullptr, combiner);
    main_graph_facade.addNode(combiner);

    NodeFacadePtr sink_p = factory.makeNode("MockupSink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    main_graph_facade.addNode(sink_p);
    std::shared_ptr<MockupSink> sink = std::dynamic_pointer_cast<MockupSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);


    // NESTED GRAPH
    NodeFacadePtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    apex_assert_hard(sub_graph);

    GraphFacade sub_graph_facade(executor, sub_graph->getGraph(), sub_graph);

    NodeFacadePtr m = factory.makeNode("DynamicMultiplier", UUIDProvider::makeUUID_without_parent("m"), sub_graph->getGraph());
    ASSERT_NE(nullptr, m);
    sub_graph_facade.addNode(m);

    apex_assert_hard(sub_graph_node_facade);
    graph->addNode(sub_graph_node_facade);

    auto type = connection_types::makeEmptyMessage<connection_types::GenericValueMessage<int> >();

    auto in1_map = sub_graph->addForwardingInput(type, "forwarding", false);
    auto in2_map = sub_graph->addForwardingInput(type, "forwarding", false);
    auto out_map = sub_graph->addForwardingOutput(type, "forwarding");

    // forwarding connections
    sub_graph_facade.connect(in1_map.internal, m, "input_a");
    sub_graph_facade.connect(in2_map.internal, m, "input_b");
    sub_graph_facade.connect(m, "output", out_map.internal);

    // top level connections
    main_graph_facade.connect(combiner, "output", sink_p, "input");

    // crossing connections
    main_graph_facade.connect(src1, "output", in1_map.external);
    main_graph_facade.connect(src2, "output", in2_map.external);
    main_graph_facade.connect(out_map.external, combiner, "input_a");
    main_graph_facade.connect(out_map.external, combiner, "input_b");

    executor.start();

    std::mutex step_done_mutex;
    std::condition_variable step_done;
    executor.end_step.connect([&]() {
        step_done.notify_all();
    });

    // execution
    std::unique_lock<std::mutex> lock(step_done_mutex);

    ASSERT_EQ(-1, sink->getValue());
    for(int iter = 0; iter < 23; ++iter) {
        sink->setWaiting(true);
        executor.step();
        sink->wait();

        step_done.wait_for(lock, std::chrono::milliseconds(50));

        ASSERT_EQ(std::pow(iter * iter, 2) , sink->getValue());
    }
}



//TEST_F(SchedulingTest, SteppingWorksForSourceGraphs) {
//    GraphFacade main_graph_facade(executor, graph, graph_node);

//    executor.setSteppingMode(true);

//    // NESTED GRAPH
//    NodeFacadePtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
//    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
//    apex_assert_hard(sub_graph);

//    GraphFacade sub_graph_facade(executor, sub_graph->getGraph(), sub_graph);

//    NodeFacadePtr src = factory.makeNode("Source", UUIDProvider::makeUUID_without_parent("src"), graph);
//    ASSERT_NE(nullptr, src);
//    sub_graph_facade.addNode(src);


//    apex_assert_hard(sub_graph_node_facade);
//    graph->addNode(sub_graph_node_facade);

//    auto type = connection_types::makeEmptyMessage<connection_types::GenericValueMessage<int> >();

//    auto out_map = sub_graph->addForwardingOutput(type, "forwarding");

//    // forwarding connections
//    sub_graph_facade.connect(src, "output", out_map.internal);

//    executor.start();

//    auto source = std::dynamic_pointer_cast<MockupSource>(src->getNode());
//    apex_assert_hard(source);

//    std::mutex step_done_mutex;
//    std::condition_variable step_done;
//    executor.end_step.connect([&]() {
//        step_done.notify_all();
//    });

//    // execution
//    std::unique_lock<std::mutex> lock(step_done_mutex);

//    for(int iter = 0; iter < 23; ++iter) {
////        ASSERT_EQ(iter, source->getValue());

//        executor.step();

//        step_done.wait_for(lock, std::chrono::milliseconds(50));

//        ASSERT_EQ(iter + 1, source->getValue());
//    }
//}
}
