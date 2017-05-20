#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/factory/node_factory.h>
#include <csapex/core/settings/settings_local.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/model/subgraph_node.h>

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

    }
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
    SubgraphNodePtr graph = std::make_shared<SubgraphNode>();
    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");
    NodeFacadePtr node = factory.makeNode("MockupNode", node_id, graph);
    graph->addNode(node);

    NodeHandle* node_found = graph->findNodeHandle(node_id);

    ASSERT_TRUE(node_found != nullptr);
    EXPECT_EQ(node_id, node_found->getUUID());
}

TEST_F(GraphTest, NodeCanBeDeleted) {
    SubgraphNodePtr graph = std::make_shared<SubgraphNode>();
    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");
    NodeFacadePtr node = factory.makeNode("MockupNode", node_id, graph);
    graph->addNode(node);

    graph->deleteNode(node_id);

    ASSERT_THROW(graph->findNode(node_id), Graph::NodeNotFoundException);
}

TEST_F(GraphTest, UnknownNodeCannotBeFound) {
    SubgraphNode graph;
    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");

    ASSERT_THROW(graph.findNode(node_id), Graph::NodeNotFoundException);
}

TEST_F(GraphTest, nullptr) {
    SubgraphNode graph;
    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");

    Node* node_found = graph.findNodeNoThrow(node_id);
    ASSERT_TRUE(node_found == nullptr);
}

}
