#ifndef NODE_CONSTRUCTING_TEST_H
#define NODE_CONSTRUCTING_TEST_H

#include <csapex/model/model_fwd.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/factory/node_wrapper.hpp>
#include <csapex/core/settings/settings_impl.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/msg/io.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/graph_facade_impl.h>

#include "gtest/gtest.h"

#include "test_exception_handler.h"
#include "mockup_nodes.h"

namespace csapex
{

class NodeConstructingTest : public ::testing::Test {
protected:
    NodeConstructingTest()
        : factory(SettingsImplementation::NoSettings, nullptr),
          executor(eh, false, false)
    {
        factory.registerNodeType(std::make_shared<NodeConstructor>("StaticMultiplier", std::bind(&NodeConstructingTest::makeStaticMultiplier<2>)));
        factory.registerNodeType(std::make_shared<NodeConstructor>("StaticMultiplier4", std::bind(&NodeConstructingTest::makeStaticMultiplier<4>)));
        factory.registerNodeType(std::make_shared<NodeConstructor>("StaticMultiplier7", std::bind(&NodeConstructingTest::makeStaticMultiplier<7>)));
        factory.registerNodeType(std::make_shared<NodeConstructor>("DynamicMultiplier", std::bind(&NodeConstructingTest::makeDynamicMultiplier)));
        factory.registerNodeType(std::make_shared<NodeConstructor>("MockupSource", std::bind(&NodeConstructingTest::makeSource)));
        factory.registerNodeType(std::make_shared<NodeConstructor>("MockupSink", std::bind(&NodeConstructingTest::makeSink)));
    }

    virtual ~NodeConstructingTest() {
    }

    virtual void SetUp() override {
        graph_node = std::make_shared<SubgraphNode>(std::make_shared<GraphImplementation>());
        graph = graph_node->getLocalGraph();

        abort_connection = error_handling::stop_request().connect([this](){
            for(graph::VertexPtr vtx : *graph) {
                NodeFacadeImplementationPtr nf = std::dynamic_pointer_cast<NodeFacadeImplementation>(vtx->getNodeFacade());
                apex_assert_hard(nf);
                if(std::shared_ptr<MockupSink> mp = std::dynamic_pointer_cast<MockupSink>(nf->getNode())) {
                    mp->abort();
                }
            }
        });
    }

    virtual void TearDown() override {
        abort_connection.disconnect();
    }

    template <int factor>
    static NodePtr makeStaticMultiplier() {
        return NodePtr(new NodeWrapper<MockupStaticMultiplierNode<factor>>());
    }
    static NodePtr makeDynamicMultiplier() {
        return NodePtr(new NodeWrapper<MockupDynamicMultiplierNode>());
    }
    static NodePtr makeSource() {
        return NodePtr(new MockupSource());
    }
    static NodePtr makeSink() {
        return NodePtr(new MockupSink());
    }

    NodeFactoryImplementation factory;
    TestExceptionHandler eh;

    ThreadPool executor;

    SubgraphNodePtr graph_node;
    GraphImplementationPtr graph;

    slim_signal::Connection abort_connection;
};
}

#endif // NODE_CONSTRUCTING_TEST_H
