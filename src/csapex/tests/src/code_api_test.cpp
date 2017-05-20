#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/factory/node_factory.h>
#include <csapex/factory/node_wrapper.hpp>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/core/settings/settings_local.h>
#include <csapex/msg/io.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/msg/direct_connection.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/core/exception_handler.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/node.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/model/subgraph_node.h>

#include "gtest/gtest.h"
#include "mockup_nodes.h"
#include "test_exception_handler.h"

#include <mutex>
#include <condition_variable>

namespace csapex {

class CodeApiTest : public ::testing::Test {
protected:

    CodeApiTest()
        : factory(SettingsLocal::NoSettings, nullptr),
          executor(eh, false, false)
    {
        factory.registerNodeType(std::make_shared<NodeConstructor>("Source", std::bind(&CodeApiTest::makeSource)));
        factory.registerNodeType(std::make_shared<NodeConstructor>("Sink", std::bind(&CodeApiTest::makeSink)));

        factory.registerNodeType(std::make_shared<NodeConstructor>("Times2", std::bind(&CodeApiTest::makeMockup<2>)));
        factory.registerNodeType(std::make_shared<NodeConstructor>("Times4", std::bind(&CodeApiTest::makeMockup<4>)));
        factory.registerNodeType(std::make_shared<NodeConstructor>("Times7", std::bind(&CodeApiTest::makeMockup<7>)));
    }

    virtual ~CodeApiTest() {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp() override {
        graph = std::make_shared<SubgraphNode>();
    }

    virtual void TearDown() override {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    template <int factor>
    static NodePtr makeMockup() {
        return NodePtr(new NodeWrapper<MockupNode<factor>>());
    }

    static NodePtr makeSource() {
        return NodePtr(new MockupSource());
    }
    static NodePtr makeSink() {
        return NodePtr(new NodeWrapper<MockupSink>());
    }


    NodeFactory factory;
    TestExceptionHandler eh;
    ThreadPool executor;

    SubgraphNodePtr graph;
};

TEST_F(CodeApiTest, GraphBuildingUsingIndices) {
    GraphFacade graph_facade(executor, graph);

    executor.setSteppingMode(true);

    NodeFacadePtr source = factory.makeNode("Source", UUIDProvider::makeUUID_without_parent("Source"), graph);
    graph_facade.addNode(source);

    NodeFacadePtr times_4 = factory.makeNode("Times4", UUIDProvider::makeUUID_without_parent("Times4"), graph);
    graph_facade.addNode(times_4);

    NodeFacadePtr times_7 = factory.makeNode("Times7", UUIDProvider::makeUUID_without_parent("Times7"), graph);
    graph_facade.addNode(times_7);

    NodeFacadePtr sink_p = factory.makeNode("Sink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    graph_facade.addNode(sink_p);

    std::shared_ptr<MockupSink> sink = std::dynamic_pointer_cast<MockupSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);

    graph_facade.connect(source, 0,
                         times_4, 0);
    graph_facade.connect(times_4, 0,
                         times_7, 0);
    graph_facade.connect(times_7, 0,
                         sink_p, 0);

    ASSERT_EQ(-1, sink->getValue());

    executor.start();

    for(int iter = 0; iter < 23; ++iter) {
        sink->setWaiting(true);
        executor.step();
        sink->wait();

//        std::cerr << "done waiting: " << iter << " // " << iter * 4 * 7 << " // " << sink->getValue() << std::endl;
        ASSERT_EQ(iter * 4 * 7, sink->getValue());
    }
}

TEST_F(CodeApiTest, GraphBuildingUsingLabels) {
    GraphFacade graph_facade(executor, graph);

    executor.setSteppingMode(true);

    NodeFacadePtr source = factory.makeNode("Source", UUIDProvider::makeUUID_without_parent("Source"), graph);
    graph_facade.addNode(source);

    NodeFacadePtr times_4 = factory.makeNode("Times4", UUIDProvider::makeUUID_without_parent("Times4"), graph);
    graph_facade.addNode(times_4);

    NodeFacadePtr times_7 = factory.makeNode("Times7", UUIDProvider::makeUUID_without_parent("Times7"), graph);
    graph_facade.addNode(times_7);

    NodeFacadePtr sink_p = factory.makeNode("Sink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    graph_facade.addNode(sink_p);

    std::shared_ptr<MockupSink> sink = std::dynamic_pointer_cast<MockupSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);

    graph_facade.connect(source, "generator",
                         times_4, "input");
    graph_facade.connect(times_4, "output",
                         times_7, "input");
    graph_facade.connect(times_7, "output",
                         sink_p, "sink");

    ASSERT_EQ(-1, sink->getValue());

    executor.start();

    for(int iter = 0; iter < 23; ++iter) {
        sink->setWaiting(true);
        executor.step();
        sink->wait();

        ASSERT_EQ(iter * 4 * 7, sink->getValue());
    }
}

TEST_F(CodeApiTest, GraphBuildingUsingUUID) {
    GraphFacade graph_facade(executor, graph);

    executor.setSteppingMode(true);

    NodeFacadePtr source = factory.makeNode("Source", UUIDProvider::makeUUID_without_parent("Source"), graph);
    graph_facade.addNode(source);

    NodeFacadePtr times_4 = factory.makeNode("Times4", UUIDProvider::makeUUID_without_parent("Times4"), graph);
    graph_facade.addNode(times_4);

    NodeFacadePtr times_7 = factory.makeNode("Times7", UUIDProvider::makeUUID_without_parent("Times7"), graph);
    graph_facade.addNode(times_7);

    NodeFacadePtr sink_p = factory.makeNode("Sink", UUIDProvider::makeUUID_without_parent("Sink"), graph);
    graph_facade.addNode(sink_p);

    std::shared_ptr<MockupSink> sink = std::dynamic_pointer_cast<MockupSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);

    graph_facade.connect(graph->makeTypedUUID_forced(source->getUUID(), "out", 0),
                         graph->makeTypedUUID_forced(times_4->getUUID(), "in", 0));
    graph_facade.connect(graph->makeTypedUUID_forced(times_4->getUUID(), "out", 0),
                         graph->makeTypedUUID_forced(times_7->getUUID(), "in", 0));
    graph_facade.connect(graph->makeTypedUUID_forced(times_7->getUUID(), "out", 0),
                         graph->makeTypedUUID_forced(sink_p->getUUID(), "in", 0));

    ASSERT_EQ(-1, sink->getValue());

    executor.start();

    for(int iter = 0; iter < 23; ++iter) {
        sink->setWaiting(true);
        executor.step();
        sink->wait();

        ASSERT_EQ(iter * 4 * 7, sink->getValue());
    }
}
}

