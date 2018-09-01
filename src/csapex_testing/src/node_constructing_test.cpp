/// HEADER
#include <csapex_testing/node_constructing_test.h>

/// PROJECT
#include <csapex/factory/node_wrapper.hpp>
#include <csapex/core/settings/settings_impl.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex_testing/test_exception_handler.h>
#include <csapex_testing/mockup_nodes.h>

using namespace csapex;

namespace csapex
{
namespace detail
{
template <int factor>
NodePtr makeStaticMultiplier()
{
    return NodePtr(new NodeWrapper<MockupStaticMultiplierNode<factor>>());
}
template <int factor>
NodePtr makeAsyncStaticMultiplier()
{
    return NodePtr(new NodeWrapper<MockupAsyncStaticMultiplierNode<factor>>());
}
template <typename T>
NodePtr makeNode()
{
    return NodePtr(new T());
}
}  // namespace detail
}  // namespace csapex

NodeConstructingTest::NodeConstructingTest()
  : node_factory(std::make_shared<NodeFactoryImplementation>(SettingsImplementation::NoSettings, nullptr)), factory(*node_factory), executor(eh, false, false, false)
{
    factory.registerNodeType(std::make_shared<NodeConstructor>("StaticMultiplier", std::bind(&detail::makeStaticMultiplier<2>)));
    factory.registerNodeType(std::make_shared<NodeConstructor>("StaticMultiplier4", std::bind(&detail::makeStaticMultiplier<4>)));
    factory.registerNodeType(std::make_shared<NodeConstructor>("AsyncStaticMultiplier4", std::bind(&detail::makeAsyncStaticMultiplier<4>)));
    factory.registerNodeType(std::make_shared<NodeConstructor>("StaticMultiplier7", std::bind(&detail::makeStaticMultiplier<7>)));
    factory.registerNodeType(std::make_shared<NodeConstructor>("DynamicMultiplier", std::bind(&detail::makeNode<NodeWrapper<MockupDynamicMultiplierNode>>)));
    factory.registerNodeType(std::make_shared<NodeConstructor>("MockupSource", std::bind(&detail::makeNode<MockupSource>)));
    factory.registerNodeType(std::make_shared<NodeConstructor>("MockupSink", std::bind(&detail::makeNode<MockupSink>)));
    factory.registerNodeType(std::make_shared<NodeConstructor>("AnySink", std::bind(&detail::makeNode<AnySink>)));
}

NodeConstructingTest::~NodeConstructingTest()
{
}

void NodeConstructingTest::SetUp()
{
    graph_node = std::make_shared<SubgraphNode>(std::make_shared<GraphImplementation>());
    graph = graph_node->getLocalGraph();

    abort_connection = error_handling::stop_request().connect([this]() {
        for (graph::VertexPtr vtx : *graph) {
            NodeFacadeImplementationPtr nf = std::dynamic_pointer_cast<NodeFacadeImplementation>(vtx->getNodeFacade());
            apex_assert_hard(nf);
            if (std::shared_ptr<MockupSink> mp = std::dynamic_pointer_cast<MockupSink>(nf->getNode())) {
                mp->abort();
            }
        }
    });
}

void NodeConstructingTest::TearDown()
{
    abort_connection.disconnect();
}
