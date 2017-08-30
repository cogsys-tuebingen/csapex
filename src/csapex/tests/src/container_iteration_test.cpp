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
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/graph/graph_local.h>

#include "gtest/gtest.h"
#include "test_exception_handler.h"
#include "stepping_test.h"

namespace csapex {

class IterationCombiner
{
public:
    IterationCombiner()
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
        //TRACEstd::cerr << " < multiply " << a << " by " << b << std::endl;
        msg::publish(output_, a * b);
    }

private:
    Input* input_a_;
    Input* input_b_;
    Output* output_;
};


class IterationSource : public Node
{
public:
    IterationSource()
        : i(0)
    {

    }

    void setup(NodeModifier& node_modifier) override
    {
        out = node_modifier.addOutput<connection_types::GenericVectorMessage, int>("output");
    }

    void setupParameters(Parameterizable& /*parameters*/) override
    {

    }


    void process() override
    {
        //TRACEstd::cerr << " < publish " << i << std::endl;
        auto vector = std::make_shared<std::vector<int>>();
        for(int j = 0; j < 8; ++j) {
            vector->push_back(j * i);
        }

        msg::publish<connection_types::GenericVectorMessage, int>(out, vector);

        ++i;
    }

private:
    Output* out;

    int i;
};

class IterationConstant : public Node
{
public:
    IterationConstant()
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
        //TRACEstd::cerr << " < publish contant " << i << std::endl;
        msg::publish(out, i);

        ++i;
    }

private:
    Output* out;

    int i;
};


class IterationSink
{
public:
    IterationSink()
    {

    }

    void setup(NodeModifier& node_modifier)
    {
        in = node_modifier.addInput<connection_types::GenericVectorMessage, int>("input");
    }

    void setupParameters(Parameterizable& /*parameters*/)
    {

    }

    void process(NodeModifier& node_modifier, Parameterizable& /*parameters*/)
    {
        auto i = msg::getMessage<connection_types::GenericVectorMessage, int>(in);
        value = *i;

        //TRACEstd::cerr << "got vector of size " << value.size() << std::endl;
    }

    std::vector<int> getValue() const
    {
        return value;
    }

private:
    Input* in;

    std::vector<int> value;
};


class ContainerIterationTest : public SteppingTest
{

protected:
    ContainerIterationTest()
    {
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("IterationCombiner",
                                                                                 std::bind(&ContainerIterationTest::makeCombiner)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("IterationSource",
                                                                                 std::bind(&ContainerIterationTest::makeSource)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("IterationConstant",
                                                                                 std::bind(&ContainerIterationTest::makeConstant)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("IterationSink",
                                                                                 std::bind(&ContainerIterationTest::makeSink)));
            factory.registerNodeType(constructor);
        }
    }

    virtual ~ContainerIterationTest() {
    }

    virtual void SetUp() override {
        SteppingTest::SetUp();

        graph_node = std::make_shared<SubgraphNode>(std::make_shared<GraphLocal>());
        graph = graph_node->getGraph();
    }


    static NodePtr makeCombiner() {
        return NodePtr(new NodeWrapper<IterationCombiner>());
    }
    static NodePtr makeSource() {
        return NodePtr(new IterationSource());
    }
    static NodePtr makeConstant() {
        return NodePtr(new IterationConstant());
    }
    static NodePtr makeSink() {
        return NodePtr(new NodeWrapper<IterationSink>());
    }
};

TEST_F(ContainerIterationTest, VectorCanBeIteratedInSubGraph) {
    GraphFacadeLocal main_graph_facade(executor, graph, graph_node);

    // MAIN GRAPH
    NodeFacadeLocalPtr src = factory.makeNode("IterationSource", UUIDProvider::makeUUID_without_parent("src"), graph);
    ASSERT_NE(nullptr, src);
    graph->addNode(src);

    NodeFacadeLocalPtr constant = factory.makeNode("IterationConstant", UUIDProvider::makeUUID_without_parent("const"), graph);
    ASSERT_NE(nullptr, constant);
    graph->addNode(constant);

    NodeFacadeLocalPtr sink_p = factory.makeNode("IterationSink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    main_graph_facade.addNode(sink_p);
    std::shared_ptr<IterationSink> sink = std::dynamic_pointer_cast<IterationSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);


    // NESTED GRAPH
    NodeFacadeLocalPtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    apex_assert_hard(sub_graph);

    GraphFacadeLocal sub_graph_facade(executor, sub_graph->getGraph(), sub_graph);

    NodeFacadeLocalPtr n2 = factory.makeNode("IterationCombiner", UUIDProvider::makeUUID_without_parent("n2"), sub_graph->getGraph());
    ASSERT_NE(nullptr, n2);
    sub_graph_facade.addNode(n2);

    apex_assert_hard(sub_graph_node_facade);
    graph->addNode(sub_graph_node_facade);

    auto type = connection_types::GenericVectorMessage::make<int>();

    auto in_vec_map = sub_graph->addForwardingInput(type, "forwarding_vector", false);
    auto in_const_map = sub_graph->addForwardingInput(connection_types::makeEmpty<connection_types::GenericValueMessage<int>>(), "forwarding_const", false);
    auto out_map = sub_graph->addForwardingOutput(type, "forwarding");

    sub_graph->setIterationEnabled(in_vec_map.external, true);

    // forwarding connections
    sub_graph_facade.connect(in_vec_map.internal, n2, "input_a");
    sub_graph_facade.connect(in_const_map.internal, n2, "input_b");
    sub_graph_facade.connect(n2, "output", out_map.internal);

    // crossing connections
    main_graph_facade.connect(src, "output", in_vec_map.external);
    main_graph_facade.connect(constant, "output", in_const_map.external);
    main_graph_facade.connect(out_map.external, sink_p, "input");

    executor.start();

    // execution
    ASSERT_TRUE(sink->getValue().empty());
    for(int iter = 0; iter < 23; ++iter) {
        step();

        std::vector<int> res = sink->getValue();

        ASSERT_EQ(8, res.size());

        int constant = iter;

        for(std::size_t j = 0; j < res.size(); ++j) {
            ASSERT_EQ(iter * j * constant, res[j]);
        }
    }
}

TEST_F(ContainerIterationTest, VectorCanBeForwardedInSubGraph) {
    GraphFacadeLocal main_graph_facade(executor, graph, graph_node);

    // MAIN GRAPH
    NodeFacadeLocalPtr src = factory.makeNode("IterationSource", UUIDProvider::makeUUID_without_parent("src"), graph);
    ASSERT_NE(nullptr, src);
    graph->addNode(src);
    NodeFacadeLocalPtr sink_p = factory.makeNode("IterationSink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    main_graph_facade.addNode(sink_p);
    std::shared_ptr<IterationSink> sink = std::dynamic_pointer_cast<IterationSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);


    // NESTED GRAPH
    NodeFacadeLocalPtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    apex_assert_hard(sub_graph);

    GraphFacadeLocal sub_graph_facade(executor, sub_graph->getGraph(), sub_graph);

    apex_assert_hard(sub_graph_node_facade);
    graph->addNode(sub_graph_node_facade);

    auto type = connection_types::GenericVectorMessage::make<int>();

    auto in_vec_map = sub_graph->addForwardingInput(type, "forwarding_vector", false);
    auto out_map = sub_graph->addForwardingOutput(type, "forwarding");

    sub_graph->setIterationEnabled(in_vec_map.external, true);

    // forwarding connections
    sub_graph_facade.connect(in_vec_map.internal,out_map.internal);

    // crossing connections
    main_graph_facade.connect(src, "output", in_vec_map.external);
    main_graph_facade.connect(out_map.external, sink_p, "input");

    executor.start();

    // execution
    ASSERT_TRUE(sink->getValue().empty());
    for(int iter = 0; iter < 23; ++iter) {
        step();

        std::vector<int> res = sink->getValue();

        ASSERT_EQ(8, res.size());

        for(std::size_t j = 0; j < res.size(); ++j) {
            ASSERT_EQ(iter * j, res[j]);
        }
    }
}

TEST_F(ContainerIterationTest, VectorCanBeForwardedAndIteratedInSubGraph) {
    GraphFacadeLocal main_graph_facade(executor, graph, graph_node);

    // MAIN GRAPH
    NodeFacadeLocalPtr src = factory.makeNode("IterationSource", UUIDProvider::makeUUID_without_parent("src"), graph);
    ASSERT_NE(nullptr, src);
    graph->addNode(src);

    NodeFacadeLocalPtr constant = factory.makeNode("IterationConstant", UUIDProvider::makeUUID_without_parent("const"), graph);
    ASSERT_NE(nullptr, constant);
    graph->addNode(constant);

    NodeFacadeLocalPtr sink_p = factory.makeNode("IterationSink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    main_graph_facade.addNode(sink_p);
    std::shared_ptr<IterationSink> sink = std::dynamic_pointer_cast<IterationSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);


    // NESTED GRAPH
    NodeFacadeLocalPtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    apex_assert_hard(sub_graph);

    GraphFacadeLocal sub_graph_facade(executor, sub_graph->getGraph(), sub_graph);

    NodeFacadeLocalPtr n2 = factory.makeNode("IterationCombiner", UUIDProvider::makeUUID_without_parent("n2"), sub_graph->getGraph());
    ASSERT_NE(nullptr, n2);
    sub_graph_facade.addNode(n2);

    apex_assert_hard(sub_graph_node_facade);
    graph->addNode(sub_graph_node_facade);

    auto type = connection_types::GenericVectorMessage::make<int>();

    auto in_vec_map = sub_graph->addForwardingInput(type, "forwarding_vector", false);
    auto in_const_map = sub_graph->addForwardingInput(connection_types::makeEmpty<connection_types::GenericValueMessage<int>>(), "forwarding_const", false);
    auto out_map = sub_graph->addForwardingOutput(type, "forwarding");

    sub_graph->setIterationEnabled(in_vec_map.external, true);

    // forwarding connections
    sub_graph_facade.connect(in_vec_map.internal, n2, "input_a"); //   these
    sub_graph_facade.connect(in_const_map.internal, n2, "input_b"); // two
//                                                                       make problems... need one more step than necessary...
    sub_graph_facade.connect(in_vec_map.internal, out_map.internal);

    // crossing connections
    main_graph_facade.connect(src, "output", in_vec_map.external);
    main_graph_facade.connect(constant, "output", in_const_map.external);
    main_graph_facade.connect(out_map.external, sink_p, "input");

    executor.start();

    // execution
    ASSERT_TRUE(sink->getValue().empty());
    for(int iter = 0; iter < 23; ++iter) {
        step();

        std::vector<int> res = sink->getValue();

        ASSERT_EQ(8, res.size());

        for(std::size_t j = 0; j < res.size(); ++j) {
            ASSERT_EQ(iter * j, res[j]);
        }
    }
}
}
