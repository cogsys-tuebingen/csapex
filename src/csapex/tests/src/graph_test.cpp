#include <csapex/model/graph/graph_local.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/factory/node_factory.h>
#include <csapex/core/settings/settings_local.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/graph/graph_local.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/input.h>

#include "gtest/gtest.h"

namespace csapex {

class MockupNode : public Node
{
public:
    MockupNode()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        test_input = node_modifier.addInput<csapex::connection_types::AnyMessage>("test");
    }

    Input* test_input;
};

class GraphTest : public ::testing::Test {
protected:
    NodeFactory factory;

    GraphTest()
        : factory(SettingsLocal::NoSettings, nullptr)
    {
        std::vector<TagPtr> tags;
        csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("MockupNode",
                                                     std::bind(&GraphTest::makeMockup)));
        factory.registerNodeType(constructor);
    }

    virtual ~GraphTest() {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp() override {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    virtual void TearDown() override {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    static NodePtr makeMockup() {
        return NodePtr(new MockupNode);
    }

};

TEST_F(GraphTest, NodeCanBeFound) {
    SubgraphNodePtr graph_node = std::make_shared<SubgraphNode>(std::make_shared<GraphLocal>());
    GraphLocalPtr graph = graph_node->getLocalGraph();
    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");
    NodeFacadeLocalPtr node = factory.makeNode("MockupNode", node_id, graph);
    graph->addNode(node);

    Node* node_found = graph->findNode(node_id);
    ASSERT_NE(nullptr, node_found);
    EXPECT_EQ(node->getNode().get(), node_found);

    NodeHandle* node_handle_found = graph->findNodeHandle(node_id);
    ASSERT_NE(nullptr, node_handle_found);
    EXPECT_EQ(node_id, node_handle_found->getUUID());
    EXPECT_EQ(node->getNodeHandle().get(), node_handle_found);

    NodeFacadePtr node_facade_found = graph->findNodeFacade(node_id);
    ASSERT_NE(nullptr, node_facade_found);
    EXPECT_EQ(node_id, node_facade_found->getUUID());
    EXPECT_EQ(node, node_facade_found);
}

TEST_F(GraphTest, NodeCanBeFoundWithAConnector) {
    SubgraphNodePtr graph_node = std::make_shared<SubgraphNode>(std::make_shared<GraphLocal>());
    GraphLocalPtr graph = graph_node->getLocalGraph();
    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");
    NodeFacadeLocalPtr node = factory.makeNode("MockupNode", node_id, graph);
    graph->addNode(node);

    //UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");
    MockupNode* mnode = dynamic_cast<MockupNode*>(node->getNode().get());
    ASSERT_NE(nullptr, mnode);

    UUID search_node_id = mnode->test_input->getUUID();
    ASSERT_NE(UUID::NONE, search_node_id);

    Node* node_found = graph->findNodeForConnector(search_node_id);
    ASSERT_NE(nullptr, node_found);
    EXPECT_EQ(node->getNode().get(), node_found);

    NodeHandle* node_handle_found = graph->findNodeHandleForConnector(search_node_id);
    ASSERT_NE(nullptr, node_handle_found);
    EXPECT_EQ(node_id, node_handle_found->getUUID());
    EXPECT_EQ(node->getNodeHandle().get(), node_handle_found);

    NodeFacadePtr node_facade_found = graph->findNodeFacadeForConnector(search_node_id);
    ASSERT_NE(nullptr, node_facade_found);
    EXPECT_EQ(node_id, node_facade_found->getUUID());
    EXPECT_EQ(node, node_facade_found);
}

TEST_F(GraphTest, NodeCanBeDeleted) {
    SubgraphNodePtr graph_node = std::make_shared<SubgraphNode>(std::make_shared<GraphLocal>());
    GraphLocalPtr graph = graph_node->getLocalGraph();
    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");
    NodeFacadeLocalPtr node = factory.makeNode("MockupNode", node_id, graph);
    graph->addNode(node);

    graph->deleteNode(node_id);

    ASSERT_THROW(graph->findNode(node_id), Graph::NodeNotFoundException);
}

TEST_F(GraphTest, UnknownNodeCannotBeFound) {
    SubgraphNode graph_node(std::make_shared<GraphLocal>());
    GraphPtr graph = graph_node.getGraph();
    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");

    ASSERT_THROW(graph->findNode(node_id), Graph::NodeNotFoundException);
}

TEST_F(GraphTest, nullptr) {
    SubgraphNode graph_node(std::make_shared<GraphLocal>());
    GraphPtr graph = graph_node.getGraph();
    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");

    NodeFacadePtr node_facade_found = graph->findNodeFacadeNoThrow(node_id);
    ASSERT_EQ(nullptr, node_facade_found);

    NodeHandle* node_handle_found = graph->findNodeHandleNoThrow(node_id);
    ASSERT_EQ(nullptr, node_handle_found);

    Node* node_found = graph->findNodeNoThrow(node_id);
    ASSERT_EQ(nullptr, node_found);
}



TEST_F(GraphTest, NestedNodeCanBeFound) {
    SubgraphNodePtr main_graph_node = std::make_shared<SubgraphNode>(std::make_shared<GraphLocal>());
    GraphLocalPtr graph = main_graph_node->getLocalGraph();

    NodeFacadeLocalPtr sub_graph_node_facade = factory.makeNode("csapex::Graph", graph->generateUUID("subgraph"), graph);
    SubgraphNodePtr sub_graph = std::dynamic_pointer_cast<SubgraphNode>(sub_graph_node_facade->getNode());
    graph->addNode(sub_graph_node_facade);

    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");
    NodeFacadeLocalPtr node_facade = factory.makeNode("MockupNode", node_id, sub_graph->getLocalGraph());
    sub_graph->getLocalGraph()->addNode(node_facade);

    NodeHandle* node_handle_found = graph->findNodeHandleNoThrow(node_id);
    ASSERT_EQ(nullptr, node_handle_found);

    node_handle_found = sub_graph->getLocalGraph()->findNodeHandle(node_id);
    ASSERT_NE(nullptr, node_handle_found);


    UUID nested_id = UUIDProvider::makeUUID_without_parent("subgraph_0" + UUID::namespace_separator + "foobarbaz");

    Node* node_found = graph->findNode(nested_id);
    ASSERT_NE(nullptr, node_found);

    EXPECT_EQ(node_id, node_found->getUUID());
    EXPECT_EQ(node_found, node_facade->getNode().get());


    node_handle_found = graph->findNodeHandle(nested_id);
    ASSERT_NE(nullptr, node_handle_found);

    EXPECT_EQ(node_id, node_handle_found->getUUID());
    EXPECT_EQ(node_handle_found, node_facade->getNodeHandle().get());


    NodeFacadePtr node_facade_found = graph->findNodeFacade(nested_id);
    ASSERT_NE(nullptr, node_facade_found);

    EXPECT_EQ(node_id, node_facade_found->getUUID());
    EXPECT_EQ(node_facade_found, node_facade);
}

TEST_F(GraphTest, RootCanBeFound) {
    UUIDProviderPtr root_provider(new UUIDProvider);
    NodeFacadeLocalPtr main_facade = factory.makeGraph(root_provider->generateUUID("graph"), root_provider);

    SubgraphNodePtr graph_node = std::dynamic_pointer_cast<SubgraphNode>(main_facade->getNode());
    ASSERT_NE(nullptr, graph_node);

    GraphPtr graph = graph_node->getGraph();

    UUID node_id = UUIDProvider::makeUUID_without_parent("~");

    Node* node_found = graph->findNode(node_id);
    ASSERT_NE(nullptr, node_found);
    EXPECT_EQ(graph_node->getUUID(), node_found->getUUID());
    EXPECT_EQ(graph_node.get(), node_found);

    NodeHandle* nodehandle_found = graph->findNodeHandle(node_id);
    ASSERT_NE(nullptr, nodehandle_found);
    EXPECT_EQ(graph_node->getUUID(), nodehandle_found->getUUID());
    EXPECT_EQ(graph_node->getNodeHandle(), nodehandle_found);

    NodeFacadePtr  nodefacade_found = graph->findNodeFacade(node_id);
    ASSERT_NE(nullptr, nodefacade_found);
    EXPECT_EQ(main_facade->getUUID(), nodefacade_found->getUUID());
    EXPECT_EQ(main_facade, nodefacade_found);

}


}
