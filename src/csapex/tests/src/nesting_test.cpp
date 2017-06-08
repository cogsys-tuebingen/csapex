#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/node.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/factory/node_factory.h>
#include <csapex/factory/node_wrapper.hpp>
#include <csapex/core/settings/settings_local.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/scheduling/thread_group.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/core/exception_handler.h>
#include <csapex/msg/output_transition.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/model/connectable.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/graph/graph_local.h>

#include "gtest/gtest.h"
#include "test_exception_handler.h"

namespace csapex {

class Multiplier
{
public:
    Multiplier()
    {
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
        input_ = node_modifier.addInput<int>("input");
        output_ = node_modifier.addOutput<int>("output");
    }

    void setupParameters(Parameterizable& /*parameters*/)
    {

    }

    void process(NodeModifier& node_modifier, Parameterizable& /*parameters*/)
    {
        int a = msg::getValue<int>(input_);
//        std::cerr << " < multiply " << a << " by two" << std::endl;
        int v = a * 2;;
        msg::publish(output_, v);
    }

private:
    Input* input_;
    Output* output_;
};


class Multiplier2
{
public:
    Multiplier2()
    {
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
        input_a_ = node_modifier.addInput<int>("input_a");
        input_b_ = node_modifier.addInput<int>("input_b");
        output_ = node_modifier.addOutput<int>("output");
    }

    void setupParameters(Parameterizable& /*parameters*/)
    {

    }

    void process(NodeModifier& node_modifier, Parameterizable& /*parameters*/)
    {
        int a = msg::getValue<int>(input_a_);
        int b = msg::getValue<int>(input_b_);
//        std::cerr << " < multiply " << a << " by " << b << std::endl;
        msg::publish(output_, a * b);
    }

private:
    Input* input_a_;
    Input* input_b_;
    Output* output_;
};


class MultiplierSource : public Node
{
public:
    MultiplierSource()
        : i(0)
    {

    }

    void setup(NodeModifier& node_modifier) override
    {
        out = node_modifier.addOutput<int>("output");
    }

    void setupParameters(Parameterizable& /*parameters*/) override
    {

    }


    void process() override
    {
       // std::cerr << " < publish " << i << std::endl;
        msg::publish(out, i++);
    }

private:
    Output* out;

    int i;
};


class MultiplierSink
{
public:
    MultiplierSink()
        : aborted(false),
          waiting(false), value(-1)
    {

    }

    void setup(NodeModifier& node_modifier)
    {
        in = node_modifier.addInput<int>("input");
        in->setEssential(true);
    }

    void setupParameters(Parameterizable& /*parameters*/)
    {

    }

    void process(NodeModifier& node_modifier, Parameterizable& /*parameters*/)
    {
        int i = msg::getValue<int>(in);
      //  std::cerr << " < sink " << i << std::endl;
        value = i;

        std::unique_lock<std::recursive_mutex> lock(wait_mutex);
        waiting = false;
        stepping_done.notify_all();
    }

    int getValue() const
    {
        return value;
    }

    void setWaiting(bool w)
    {
        waiting = w;
    }

    void wait()
    {
        if(aborted) {
            return;
        }
        std::unique_lock<std::recursive_mutex> lock(wait_mutex);
        while(waiting) {
            stepping_done.wait(lock);
        }
    }

    void abort()
    {
        aborted = true;

        std::unique_lock<std::recursive_mutex> lock(wait_mutex);
        if(waiting) {
            waiting = false;
            stepping_done.notify_all();
        }
    }

private:
    Input* in;

    bool aborted;
    bool waiting;

    std::recursive_mutex wait_mutex;
    std::condition_variable_any stepping_done;

    int value;
};


class NestingTest : public ::testing::Test {
protected:
    NestingTest()
        : factory(SettingsLocal::NoSettings, nullptr),
          executor(eh, false, false)
    {
        std::vector<TagPtr> tags;
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("Multiplier",
                                                                                 std::bind(&NestingTest::makeMockup)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("Multiplier2",
                                                                                 std::bind(&NestingTest::makeMockup2)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("MultiplierSource",
                                                                                 std::bind(&NestingTest::makeSource)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("MultiplierSink",
                                                                                 std::bind(&NestingTest::makeSink)));
            factory.registerNodeType(constructor);
        }
    }

    virtual ~NestingTest() {
    }

    virtual void SetUp() override {
        graph_node = std::make_shared<SubgraphNode>(std::make_shared<GraphLocal>());
        graph = graph_node->getGraph();

        abort_connection = error_handling::stop_request().connect([this](){
            for(auto it = graph->begin(); it != graph->end(); ++it) {
                NodeFacadePtr nf = (*it)->getNodeFacade();
                if(std::shared_ptr<MultiplierSink> mp = std::dynamic_pointer_cast<MultiplierSink>(nf->getNode())) {
                    mp->abort();
                }
            }
        });
    }

    virtual void TearDown() override {
        abort_connection.disconnect();
    }

    static NodePtr makeMockup() {
        return NodePtr(new NodeWrapper<Multiplier>());
    }
    static NodePtr makeMockup2() {
        return NodePtr(new NodeWrapper<Multiplier2>());
    }
    static NodePtr makeSource() {
        return NodePtr(new MultiplierSource());
    }
    static NodePtr makeSink() {
        return NodePtr(new NodeWrapper<MultiplierSink>());
    }

    NodeFactory factory;
    TestExceptionHandler eh;
    
    ThreadPool executor;

    SubgraphNodePtr graph_node;
    GraphPtr graph;

    slim_signal::Connection abort_connection;
};


TEST_F(NestingTest, NodesCanBeGroupedIntoSubgraphWithOneExecutor) {
    GraphFacade main_graph_facade(executor, graph, graph_node);

    executor.setSteppingMode(true);

    // MAIN GRAPH
    NodeFacadePtr src = factory.makeNode("MultiplierSource", UUIDProvider::makeUUID_without_parent("src"), graph);
    ASSERT_NE(nullptr, src);
    graph->addNode(src);

    NodeFacadePtr sink_p = factory.makeNode("MultiplierSink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    main_graph_facade.addNode(sink_p);
    std::shared_ptr<MultiplierSink> sink = std::dynamic_pointer_cast<MultiplierSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);


    // NESTED GRAPH
    NodeFacadePtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    apex_assert_hard(sub_graph);
    GraphFacade sub_graph_facade(executor, sub_graph->getGraph(), sub_graph);

    NodeFacadePtr n2 = factory.makeNode("Multiplier", UUIDProvider::makeUUID_without_parent("n2"), sub_graph->getGraph());
    ASSERT_NE(nullptr, n2);
    sub_graph_facade.addNode(n2);
    NodeFacadePtr n3 = factory.makeNode("Multiplier2", UUIDProvider::makeUUID_without_parent("n3"), sub_graph->getGraph());
    ASSERT_NE(nullptr, n3);
    sub_graph_facade.addNode(n3);
    NodeFacadePtr src2 = factory.makeNode("MultiplierSource", UUIDProvider::makeUUID_without_parent("src2"), sub_graph->getGraph());
    ASSERT_NE(nullptr, src);
    sub_graph_facade.addNode(src2);
    NodeFacadePtr n4 = factory.makeNode("Multiplier", UUIDProvider::makeUUID_without_parent("n4"), sub_graph->getGraph());
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
        sink->setWaiting(true);
        executor.step();
        sink->wait();

        ASSERT_EQ(iter * std::pow(2, 2) * iter , sink->getValue());
    }
}

TEST_F(NestingTest, NodesCanBeGroupedIntoSubgraphWithSeparateExecutors) {
    GraphFacade main_graph_facade(executor, graph, graph_node);

    executor.setSteppingMode(true);

    // MAIN GRAPH
    NodeFacadePtr src = factory.makeNode("MultiplierSource", UUIDProvider::makeUUID_without_parent("src"), graph);
    ASSERT_NE(nullptr, src);
    graph->addNode(src);

    NodeFacadePtr sink_p = factory.makeNode("MultiplierSink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    main_graph_facade.addNode(sink_p);
    std::shared_ptr<MultiplierSink> sink = std::dynamic_pointer_cast<MultiplierSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);


    // NESTED GRAPH
    NodeFacadePtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    apex_assert_hard(sub_graph);

    ThreadPool sub_executor(&executor, eh, executor.isThreadingEnabled(), executor.isGroupingEnabled());
    GraphFacade sub_graph_facade(sub_executor, sub_graph->getGraph(), sub_graph);

    NodeFacadePtr n2 = factory.makeNode("Multiplier", UUIDProvider::makeUUID_without_parent("n2"), sub_graph->getGraph());
    ASSERT_NE(nullptr, n2);
    sub_graph_facade.addNode(n2);
    NodeFacadePtr n3 = factory.makeNode("Multiplier2", UUIDProvider::makeUUID_without_parent("n3"), sub_graph->getGraph());
    ASSERT_NE(nullptr, n3);
    sub_graph_facade.addNode(n3);
    NodeFacadePtr src2 = factory.makeNode("MultiplierSource", UUIDProvider::makeUUID_without_parent("src2"), sub_graph->getGraph());
    ASSERT_NE(nullptr, src);
    sub_graph_facade.addNode(src2);
    NodeFacadePtr n4 = factory.makeNode("Multiplier", UUIDProvider::makeUUID_without_parent("n4"), sub_graph->getGraph());
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
        sink->setWaiting(true);
        executor.step();
        sink->wait();

        ASSERT_EQ(iter * std::pow(2, 2) * iter , sink->getValue());
    }
}

TEST_F(NestingTest, SubgraphWithMultipleInputsAndOutputs) {
    GraphFacade main_graph_facade(executor, graph, graph_node);

    executor.setSteppingMode(true);

    // MAIN GRAPH
    NodeFacadePtr src1 = factory.makeNode("MultiplierSource", UUIDProvider::makeUUID_without_parent("src1"), graph);
    ASSERT_NE(nullptr, src1);
    main_graph_facade.addNode(src1);

    NodeFacadePtr src2 = factory.makeNode("MultiplierSource", UUIDProvider::makeUUID_without_parent("src2"), graph);
    ASSERT_NE(nullptr, src2);
    main_graph_facade.addNode(src2);


    NodeFacadePtr combiner = factory.makeNode("Multiplier2", UUIDProvider::makeUUID_without_parent("combiner"), graph);
    ASSERT_NE(nullptr, combiner);
    main_graph_facade.addNode(combiner);

    NodeFacadePtr sink_p = factory.makeNode("MultiplierSink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    main_graph_facade.addNode(sink_p);
    std::shared_ptr<MultiplierSink> sink = std::dynamic_pointer_cast<MultiplierSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);


    // NESTED GRAPH
    NodeFacadePtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    apex_assert_hard(sub_graph);

    GraphFacade sub_graph_facade(executor, sub_graph->getGraph(), sub_graph);

    NodeFacadePtr m = factory.makeNode("Multiplier2", UUIDProvider::makeUUID_without_parent("m"), sub_graph->getGraph());
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
        sink->setWaiting(true);
        executor.step();
        sink->wait();

        ASSERT_EQ(std::pow(iter * iter, 2) , sink->getValue());
    }
}



TEST_F(NestingTest, NestedUUIDs) {
    GraphFacade main_graph_facade(executor, graph, graph_node);

    executor.setSteppingMode(true);

    // MAIN GRAPH
    NodeFacadePtr m1 = factory.makeNode("Multiplier", graph->generateUUID("src"), graph);
    ASSERT_NE(nullptr, m1);
    graph->addNode(m1);

    NodeHandle* m1_found = graph->findNodeHandleNoThrow(UUIDProvider::makeUUID_without_parent("src_0"));
    ASSERT_NE(nullptr, m1_found);
    ASSERT_EQ(m1->getNodeHandle().get(), m1_found);

    // NESTED GRAPH
    UUID sub_graph_1_uuid = graph->generateUUID("subgraph");
    NodeFacadePtr sub_graph_1_handle = factory.makeNode("csapex::Graph", sub_graph_1_uuid, graph);
    ASSERT_NE(nullptr, sub_graph_1_handle);
    main_graph_facade.addNode(sub_graph_1_handle);

    UUID sub_graph_2_uuid = graph->generateUUID("subgraph");
    NodeFacadePtr sub_graph_2_handle = factory.makeNode("csapex::Graph", sub_graph_2_uuid, graph);
    ASSERT_NE(nullptr, sub_graph_2_handle);
    main_graph_facade.addNode(sub_graph_2_handle);

    GraphFacade* sub_graph_2_facade_ptr = main_graph_facade.getSubGraph(sub_graph_2_uuid);
    ASSERT_NE(nullptr, sub_graph_2_facade_ptr);
    GraphFacade& sub_graph_2_facade = *sub_graph_2_facade_ptr;

    GraphPtr sub_graph_2 = sub_graph_2_facade_ptr->getGraph();


    NodeFacadePtr m2 = factory.makeNode("Multiplier", sub_graph_2->generateUUID("src"), sub_graph_2);
    ASSERT_NE(nullptr, m2);
    sub_graph_2_facade.addNode(m2);


    // FURTHER LEVEL
    UUID sub_graph_3_uuid = sub_graph_2->generateUUID("subgraph");
    NodeFacadePtr sub_graph_3_handle = factory.makeNode("csapex::Graph", sub_graph_3_uuid, sub_graph_2);
    ASSERT_NE(nullptr, sub_graph_3_handle);
    sub_graph_2_facade.addNode(sub_graph_3_handle);

    GraphFacade* sub_graph_3_facade_ptr = sub_graph_2_facade.getSubGraph(sub_graph_3_uuid);
    ASSERT_NE(nullptr, sub_graph_3_facade_ptr);
    GraphFacade& sub_graph_3_facade = *sub_graph_3_facade_ptr;

    GraphPtr sub_graph_3 = sub_graph_3_facade_ptr->getGraph();


    NodeFacadePtr m3 = factory.makeNode("Multiplier", sub_graph_3->generateUUID("src"), sub_graph_3);
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
    GraphFacade main_graph_facade(executor, graph, graph_node);

    executor.setSteppingMode(true);

    // MAIN GRAPH
    NodeFacadePtr sink_p = factory.makeNode("MultiplierSink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    main_graph_facade.addNode(sink_p);
    std::shared_ptr<MultiplierSink> sink = std::dynamic_pointer_cast<MultiplierSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);


    // NESTED GRAPH
    NodeFacadePtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    apex_assert_hard(sub_graph);

    GraphFacade sub_graph_facade(executor, sub_graph->getGraph(), sub_graph);

    NodeFacadePtr src = factory.makeNode("MultiplierSource", UUIDProvider::makeUUID_without_parent("src"), graph);
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

    // execution
    ASSERT_EQ(-1, sink->getValue());
    for(int iter = 0; iter < 23; ++iter) {
        sink->setWaiting(true);
        executor.step();
        sink->wait();

        ASSERT_EQ(iter , sink->getValue());
    }
}
}
