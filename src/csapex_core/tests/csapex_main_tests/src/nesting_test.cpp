#include <csapex/model/graph_facade.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/test/mockup_nodes.h>
#include <csapex/test/stepping_test.h>

namespace csapex {

class NestingTest : public SteppingTest
{

};

TEST_F(NestingTest, InternalGraphPortsCanBeFound) {
    GraphFacadeImplementation main_graph_facade(executor, graph, graph_node);

    UUID subgraph_id = graph->generateUUID("subgraph");

    NodeFacadeImplementationPtr sub_graph_node_facade = factory.makeNode("csapex::Graph", subgraph_id, graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    apex_assert_hard(sub_graph);
    GraphFacadeImplementation sub_graph_facade(executor, sub_graph->getLocalGraph(), sub_graph);

    main_graph_facade.addNode(sub_graph_node_facade);

    SubgraphNodePtr sub_graph_node = sub_graph_facade.getSubgraphNode();
    TokenDataConstPtr type(new connection_types::AnyMessage);
    sub_graph_node->addForwardingOutput(type, "relay");
    sub_graph_node->addForwardingInput(type, "relay", false);
    sub_graph_node->addForwardingSlot(type, "relay");
    sub_graph_node->addForwardingEvent(type, "relay");

    UUID subgraph_id_expected = UUIDProvider::makeUUID_without_parent("subgraph_0");
    NodeFacadePtr nf = main_graph_facade.findNodeFacadeNoThrow(subgraph_id_expected);
    ASSERT_NE(nullptr, nf);

    {
        UUID relay_in_expected = UUIDProvider::makeUUID_without_parent("subgraph_0:|:relayin_0");
        ASSERT_NE(nullptr, main_graph_facade.findConnector(relay_in_expected));

        UUID relay_out_expected = UUIDProvider::makeUUID_without_parent("subgraph_0:|:relayout_0");
        ASSERT_NE(nullptr, main_graph_facade.findConnector(relay_out_expected));

        UUID relay_slot_expected = UUIDProvider::makeUUID_without_parent("subgraph_0:|:relayslot_0");
        ASSERT_NE(nullptr, main_graph_facade.findConnector(relay_slot_expected));

        UUID relay_evevent_expected = UUIDProvider::makeUUID_without_parent("subgraph_0:|:relayevent_0");
        ASSERT_NE(nullptr, main_graph_facade.findConnector(relay_evevent_expected));
    }
    {
        UUID relay_in_expected = UUIDProvider::makeUUID_without_parent("relayin_0");
        ASSERT_NE(nullptr, sub_graph_facade.findConnector(relay_in_expected));

        UUID relay_out_expected = UUIDProvider::makeUUID_without_parent("relayout_0");
        ASSERT_NE(nullptr, sub_graph_facade.findConnector(relay_out_expected));

        UUID relay_slot_expected = UUIDProvider::makeUUID_without_parent("relayslot_0");
        ASSERT_NE(nullptr, sub_graph_facade.findConnector(relay_slot_expected));

        UUID relay_evevent_expected = UUIDProvider::makeUUID_without_parent("relayevent_0");
        ASSERT_NE(nullptr, sub_graph_facade.findConnector(relay_evevent_expected));
    }
}

TEST_F(NestingTest, NodesCanBeGroupedIntoSubgraphWithOneExecutor) {
    GraphFacadeImplementation main_graph_facade(executor, graph, graph_node);

    // MAIN GRAPH
    NodeFacadeImplementationPtr src = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src"), graph);
    ASSERT_NE(nullptr, src);
    graph->addNode(src);

    NodeFacadeImplementationPtr sink_p = factory.makeNode("MockupSink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    main_graph_facade.addNode(sink_p);
    std::shared_ptr<MockupSink> sink = std::dynamic_pointer_cast<MockupSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);


    // NESTED GRAPH
    NodeFacadeImplementationPtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    apex_assert_hard(sub_graph);
    GraphFacadeImplementation sub_graph_facade(executor, sub_graph->getLocalGraph(), sub_graph);

    NodeFacadeImplementationPtr n2 = factory.makeNode("StaticMultiplier", UUIDProvider::makeUUID_without_parent("n2"), sub_graph->getLocalGraph());
    ASSERT_NE(nullptr, n2);
    sub_graph_facade.addNode(n2);
    NodeFacadeImplementationPtr n3 = factory.makeNode("DynamicMultiplier", UUIDProvider::makeUUID_without_parent("n3"), sub_graph->getLocalGraph());
    ASSERT_NE(nullptr, n3);
    sub_graph_facade.addNode(n3);
    NodeFacadeImplementationPtr src2 = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src2"), sub_graph->getLocalGraph());
    ASSERT_NE(nullptr, src);
    sub_graph_facade.addNode(src2);
    NodeFacadeImplementationPtr n4 = factory.makeNode("StaticMultiplier", UUIDProvider::makeUUID_without_parent("n4"), sub_graph->getLocalGraph());
    ASSERT_NE(nullptr, n4);
    sub_graph_facade.addNode(n4);


    apex_assert_hard(sub_graph_node_facade);
    graph->addNode(sub_graph_node_facade);

    auto type = connection_types::makeEmptyMessage<connection_types::GenericValueMessage<int> >();

    auto in_map = sub_graph->addForwardingInput(type, "forwarding", false);
    auto out_map = sub_graph->addForwardingOutput(type, "forwarding");

    // forwarding connections
    sub_graph_facade.connect(in_map.internal, n2, "input");
    sub_graph_facade.connect(n4, "output", out_map.internal);

    // internal sub graph connections
    sub_graph_facade.connect(n2, "output", n3, "input_a");
    sub_graph_facade.connect(src2, "output", n3, "input_b");
    sub_graph_facade.connect(n3, "output", n4, "input");

    // crossing connections
    main_graph_facade.connect(src, "output", in_map.external);
    main_graph_facade.connect(out_map.external, sink_p, "input");

    executor.start();

    // execution
    ASSERT_EQ(-1, sink->getValue());
    for(int iter = 0; iter < 23; ++iter) {
        ASSERT_NO_FATAL_FAILURE(step());

        ASSERT_EQ(iter * std::pow(2, 2) * iter , sink->getValue());
    }
}

TEST_F(NestingTest, NodesCanBeGroupedIntoSubgraphWithSeparateExecutors) {
    GraphFacadeImplementation main_graph_facade(executor, graph, graph_node);

    // MAIN GRAPH
    NodeFacadeImplementationPtr src = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src"), graph);
    ASSERT_NE(nullptr, src);
    graph->addNode(src);

    NodeFacadeImplementationPtr sink_p = factory.makeNode("MockupSink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    main_graph_facade.addNode(sink_p);
    std::shared_ptr<MockupSink> sink = std::dynamic_pointer_cast<MockupSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);


    // NESTED GRAPH
    NodeFacadeImplementationPtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    apex_assert_hard(sub_graph);

    ThreadPool sub_executor(&executor, eh, executor.isThreadingEnabled(), executor.isGroupingEnabled());
    GraphFacadeImplementation sub_graph_facade(sub_executor, sub_graph->getLocalGraph(), sub_graph);

    NodeFacadeImplementationPtr n2 = factory.makeNode("StaticMultiplier", UUIDProvider::makeUUID_without_parent("n2"), sub_graph->getLocalGraph());
    ASSERT_NE(nullptr, n2);
    sub_graph_facade.addNode(n2);
    NodeFacadeImplementationPtr n3 = factory.makeNode("DynamicMultiplier", UUIDProvider::makeUUID_without_parent("n3"), sub_graph->getLocalGraph());
    ASSERT_NE(nullptr, n3);
    sub_graph_facade.addNode(n3);
    NodeFacadeImplementationPtr src2 = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src2"), sub_graph->getLocalGraph());
    ASSERT_NE(nullptr, src);
    sub_graph_facade.addNode(src2);
    NodeFacadeImplementationPtr n4 = factory.makeNode("StaticMultiplier", UUIDProvider::makeUUID_without_parent("n4"), sub_graph->getLocalGraph());
    ASSERT_NE(nullptr, n4);
    sub_graph_facade.addNode(n4);

    apex_assert_hard(sub_graph_node_facade);
    graph->addNode(sub_graph_node_facade);

    auto type = connection_types::makeEmptyMessage<connection_types::GenericValueMessage<int> >();

    auto in_map = sub_graph->addForwardingInput(type, "forwarding", false);
    auto out_map = sub_graph->addForwardingOutput(type, "forwarding");

    // forwarding connections
    sub_graph_facade.connect(in_map.internal, n2, "input");
    sub_graph_facade.connect(n4, "output", out_map.internal);

    // internal sub graph connections
    sub_graph_facade.connect(n2, "output", n3, "input_a");
    sub_graph_facade.connect(src2, "output", n3, "input_b");
    sub_graph_facade.connect(n3, "output", n4, "input");

    // crossing connections
    main_graph_facade.connect(src, "output", in_map.external);
    main_graph_facade.connect(out_map.external, sink_p, "input");

    executor.start();
    sub_executor.start();

    // execution
    ASSERT_EQ(-1, sink->getValue());
    for(int iter = 0; iter < 23; ++iter) {
        ASSERT_NO_FATAL_FAILURE(step());

        ASSERT_EQ(iter * std::pow(2, 2) * iter , sink->getValue());
    }
}

TEST_F(NestingTest, SubgraphWithMultipleInputsAndOutputs) {
    GraphFacadeImplementation main_graph_facade(executor, graph, graph_node);

    // MAIN GRAPH
    NodeFacadeImplementationPtr src1 = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src1"), graph);
    ASSERT_NE(nullptr, src1);
    main_graph_facade.addNode(src1);

    NodeFacadeImplementationPtr src2 = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src2"), graph);
    ASSERT_NE(nullptr, src2);
    main_graph_facade.addNode(src2);


    NodeFacadeImplementationPtr combiner = factory.makeNode("DynamicMultiplier", UUIDProvider::makeUUID_without_parent("combiner"), graph);
    ASSERT_NE(nullptr, combiner);
    main_graph_facade.addNode(combiner);

    NodeFacadeImplementationPtr sink_p = factory.makeNode("MockupSink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    main_graph_facade.addNode(sink_p);
    std::shared_ptr<MockupSink> sink = std::dynamic_pointer_cast<MockupSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);


    // NESTED GRAPH
    NodeFacadeImplementationPtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    apex_assert_hard(sub_graph);

    GraphFacadeImplementation sub_graph_facade(executor, sub_graph->getLocalGraph(), sub_graph);

    NodeFacadeImplementationPtr m = factory.makeNode("DynamicMultiplier", UUIDProvider::makeUUID_without_parent("m"), sub_graph->getLocalGraph());
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

    // execution
    ASSERT_EQ(-1, sink->getValue());
    for(int iter = 0; iter < 23; ++iter) {
        ASSERT_NO_FATAL_FAILURE(step());

        ASSERT_EQ(std::pow(iter * iter, 2) , sink->getValue());
    }
}



TEST_F(NestingTest, NestedUUIDs) {
    GraphFacadeImplementation main_graph_facade(executor, graph, graph_node);

    // MAIN GRAPH
    NodeFacadeImplementationPtr m1 = factory.makeNode("StaticMultiplier", graph->generateUUID("src"), graph);
    ASSERT_NE(nullptr, m1);
    graph->addNode(m1);

    NodeHandle* m1_found = graph->findNodeHandleNoThrow(UUIDProvider::makeUUID_without_parent("src_0"));
    ASSERT_NE(nullptr, m1_found);
    ASSERT_EQ(m1->getNodeHandle().get(), m1_found);

    // NESTED GRAPH
    UUID sub_graph_1_uuid = graph->generateUUID("subgraph");
    NodeFacadeImplementationPtr sub_graph_1_handle = factory.makeNode("csapex::Graph", sub_graph_1_uuid, graph);
    ASSERT_NE(nullptr, sub_graph_1_handle);
    main_graph_facade.addNode(sub_graph_1_handle);

    UUID sub_graph_2_uuid = graph->generateUUID("subgraph");
    NodeFacadeImplementationPtr sub_graph_2_handle = factory.makeNode("csapex::Graph", sub_graph_2_uuid, graph);
    ASSERT_NE(nullptr, sub_graph_2_handle);
    main_graph_facade.addNode(sub_graph_2_handle);

    GraphFacadeImplementationPtr sub_graph_2_facade_ptr = std::dynamic_pointer_cast<GraphFacadeImplementation>(main_graph_facade.getSubGraph(sub_graph_2_uuid));
    ASSERT_NE(nullptr, sub_graph_2_facade_ptr);
    GraphFacadeImplementation& sub_graph_2_facade = *sub_graph_2_facade_ptr;

    GraphImplementationPtr sub_graph_2 = sub_graph_2_facade_ptr->getLocalGraph();


    NodeFacadeImplementationPtr m2 = factory.makeNode("StaticMultiplier", sub_graph_2->generateUUID("src"), sub_graph_2);
    ASSERT_NE(nullptr, m2);
    sub_graph_2_facade.addNode(m2);


    // FURTHER LEVEL
    UUID sub_graph_3_uuid = sub_graph_2->generateUUID("subgraph");
    NodeFacadeImplementationPtr sub_graph_3_handle = factory.makeNode("csapex::Graph", sub_graph_3_uuid, sub_graph_2);
    ASSERT_NE(nullptr, sub_graph_3_handle);
    sub_graph_2_facade.addNode(sub_graph_3_handle);

    GraphFacadeImplementationPtr sub_graph_3_facade_ptr = std::dynamic_pointer_cast<GraphFacadeImplementation>(sub_graph_2_facade.getSubGraph(sub_graph_3_uuid));
    ASSERT_NE(nullptr, sub_graph_3_facade_ptr);
    GraphFacadeImplementation& sub_graph_3_facade = *sub_graph_3_facade_ptr;

    GraphPtr sub_graph_3 = sub_graph_3_facade_ptr->getLocalGraph();


    NodeFacadeImplementationPtr m3 = factory.makeNode("StaticMultiplier", sub_graph_3->generateUUID("src"), sub_graph_3);
    ASSERT_NE(nullptr, m3);
    sub_graph_3_facade.addNode(m3);


    // SEARCH IN SUB GRAPH
    NodeHandle* m2_found = sub_graph_2->findNodeHandleNoThrow(UUIDProvider::makeUUID_without_parent("src_0"));
    ASSERT_NE(nullptr, m2_found);
    ASSERT_EQ(m2->getNodeHandle().get(), m2_found);

    // SEARCH LEVEL 2 FROM TOP WITH NAMESPACE
    ASSERT_EQ("subgraph_1", sub_graph_2_uuid.getFullName());
    UUID nested_2_id = UUIDProvider::makeDerivedUUID_forced(sub_graph_2_uuid, "src_0");
    ASSERT_EQ("subgraph_1:|:src_0", nested_2_id.getFullName());

    NodeHandle* m2_found_top = graph->findNodeHandleNoThrow(nested_2_id);
    ASSERT_NE(nullptr, m2_found_top);
    ASSERT_EQ(m2->getNodeHandle().get(), m2_found_top);

    // SEARCH LEVEL 3 FROM TOP WITH NAMESPACE
    ASSERT_EQ("subgraph_0", sub_graph_1_uuid.getFullName());
    ASSERT_EQ("subgraph_1", sub_graph_2_uuid.getFullName());
    ASSERT_EQ("subgraph_0", sub_graph_3_uuid.getFullName()); // IDs are generated for each graph in dependently!
    UUID nested_3_id = UUIDProvider::makeDerivedUUID_forced(UUIDProvider::makeUUID_without_parent("subgraph_1:|:subgraph_0"), "src_0");
    ASSERT_EQ("subgraph_1:|:subgraph_0:|:src_0", nested_3_id.getFullName());

    NodeHandle* m3_found_top = graph->findNodeHandleNoThrow(nested_3_id);
    ASSERT_NE(nullptr, m3_found_top);
    ASSERT_EQ(m3->getNodeHandle().get(), m3_found_top);

}

TEST_F(NestingTest, GroupCanBeSource) {
    GraphFacadeImplementation main_graph_facade(executor, graph, graph_node);

    // MAIN GRAPH
    NodeFacadeImplementationPtr sink_p = factory.makeNode("MockupSink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    main_graph_facade.addNode(sink_p);
    std::shared_ptr<MockupSink> sink = std::dynamic_pointer_cast<MockupSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);


    // NESTED GRAPH
    NodeFacadeImplementationPtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    apex_assert_hard(sub_graph);

    GraphFacadeImplementation sub_graph_facade(executor, sub_graph->getLocalGraph(), sub_graph);

    NodeFacadeImplementationPtr src = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src"), graph);
    ASSERT_NE(nullptr, src);
    sub_graph_facade.addNode(src);


    apex_assert_hard(sub_graph_node_facade);
    graph->addNode(sub_graph_node_facade);

    auto type = connection_types::makeEmptyMessage<connection_types::GenericValueMessage<int> >();

    auto out_map = sub_graph->addForwardingOutput(type, "forwarding");

    // forwarding connections
    sub_graph_facade.connect(src, "output", out_map.internal);

    // crossing connections
    main_graph_facade.connect(out_map.external, sink_p, "input");

    executor.start();

    auto source = std::dynamic_pointer_cast<MockupSource>(src->getNode());
    apex_assert_hard(source);

    // execution
    ASSERT_EQ(-1, sink->getValue());
    for(int iter = 0; iter < 10; ++iter) {
        ASSERT_NO_FATAL_FAILURE(step());

        ASSERT_EQ(iter + 1, source->getValue());
        ASSERT_EQ(iter , sink->getValue());
    }
}

TEST_F(NestingTest, GroupCanBeUnconnectedSource) {
    GraphFacadeImplementation main_graph_facade(executor, graph, graph_node);

    // NESTED GRAPH
    NodeFacadeImplementationPtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    apex_assert_hard(sub_graph);

    GraphFacadeImplementation sub_graph_facade(executor, sub_graph->getLocalGraph(), sub_graph);

    NodeFacadeImplementationPtr src = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("src"), graph);
    ASSERT_NE(nullptr, src);
    sub_graph_facade.addNode(src);


    apex_assert_hard(sub_graph_node_facade);
    graph->addNode(sub_graph_node_facade);

    auto type = connection_types::makeEmptyMessage<connection_types::GenericValueMessage<int> >();

    auto out_map = sub_graph->addForwardingOutput(type, "forwarding");

    // forwarding connections
    sub_graph_facade.connect(src, "output", out_map.internal);

    executor.start();

    auto source = std::dynamic_pointer_cast<MockupSource>(src->getNode());
    apex_assert_hard(source);

    // execution
    for(int iter = 0; iter < 10; ++iter) {
//        ASSERT_EQ(iter, source->getValue());

        ASSERT_NO_FATAL_FAILURE(step());

        ASSERT_EQ(iter + 1, source->getValue());
    }
}
}
