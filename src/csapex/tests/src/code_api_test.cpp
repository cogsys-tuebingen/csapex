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
#include <csapex/model/graph_facade_local.h>
#include <csapex/model/node.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/graph/graph_local.h>

#include "gtest/gtest.h"
#include "mockup_nodes.h"
#include "test_exception_handler.h"

#include <mutex>
#include <condition_variable>

#include "stepping_test.h"

namespace csapex {

class CodeApiTest : public SteppingTest
{

};

TEST_F(CodeApiTest, GraphBuildingUsingIndices) {
    GraphFacadeLocal graph_facade(executor, graph, graph_node);

    NodeFacadeLocalPtr source = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("MockupSource"), graph);
    graph_facade.addNode(source);

    NodeFacadeLocalPtr times_4 = factory.makeNode("StaticMultiplier4", UUIDProvider::makeUUID_without_parent("StaticMultiplier4"), graph);
    graph_facade.addNode(times_4);

    NodeFacadeLocalPtr times_7 = factory.makeNode("StaticMultiplier7", UUIDProvider::makeUUID_without_parent("StaticMultiplier7"), graph);
    graph_facade.addNode(times_7);

    NodeFacadeLocalPtr sink_p = factory.makeNode("MockupSink", UUIDProvider::makeUUID_without_parent("MockupSink"), graph);
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
        ASSERT_NO_FATAL_FAILURE(step());

//        std::cerr << "done waiting: " << iter << " // " << iter * 4 * 7 << " // " << sink->getValue() << std::endl;
        ASSERT_EQ(iter * 4 * 7, sink->getValue());
    }
}

TEST_F(CodeApiTest, GraphBuildingUsingLabels) {
    GraphFacadeLocal graph_facade(executor, graph, graph_node);

    NodeFacadeLocalPtr source = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("MockupSource"), graph);
    graph_facade.addNode(source);

    NodeFacadeLocalPtr times_4 = factory.makeNode("StaticMultiplier4", UUIDProvider::makeUUID_without_parent("StaticMultiplier4"), graph);
    graph_facade.addNode(times_4);

    NodeFacadeLocalPtr times_7 = factory.makeNode("StaticMultiplier7", UUIDProvider::makeUUID_without_parent("StaticMultiplier7"), graph);
    graph_facade.addNode(times_7);

    NodeFacadeLocalPtr sink_p = factory.makeNode("MockupSink", UUIDProvider::makeUUID_without_parent("MockupSink"), graph);
    graph_facade.addNode(sink_p);

    std::shared_ptr<MockupSink> sink = std::dynamic_pointer_cast<MockupSink>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);

    graph_facade.connect(source, "output",
                         times_4, "input");
    graph_facade.connect(times_4, "output",
                         times_7, "input");
    graph_facade.connect(times_7, "output",
                         sink_p, "input");

    ASSERT_EQ(-1, sink->getValue());

    executor.start();

    for(int iter = 0; iter < 23; ++iter) {
        ASSERT_NO_FATAL_FAILURE(step());

        ASSERT_EQ(iter * 4 * 7, sink->getValue());
    }
}

TEST_F(CodeApiTest, GraphBuildingUsingUUID) {
    GraphFacadeLocal graph_facade(executor, graph, graph_node);

    NodeFacadeLocalPtr source = factory.makeNode("MockupSource", UUIDProvider::makeUUID_without_parent("MockupSource"), graph);
    graph_facade.addNode(source);

    NodeFacadeLocalPtr times_4 = factory.makeNode("StaticMultiplier4", UUIDProvider::makeUUID_without_parent("StaticMultiplier4"), graph);
    graph_facade.addNode(times_4);

    NodeFacadeLocalPtr times_7 = factory.makeNode("StaticMultiplier7", UUIDProvider::makeUUID_without_parent("StaticMultiplier7"), graph);
    graph_facade.addNode(times_7);

    NodeFacadeLocalPtr sink_p = factory.makeNode("MockupSink", UUIDProvider::makeUUID_without_parent("MockupSink"), graph);
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
        ASSERT_NO_FATAL_FAILURE(step());

        ASSERT_EQ(iter * 4 * 7, sink->getValue());
    }
}
}

