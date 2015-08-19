#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/factory/node_factory.h>
#include <csapex/core/settings.h>
#include "gtest/gtest.h"

namespace csapex {

class MockupNode : public Node
{
public:
    MockupNode()
    {
    }

    void setup()
    {
    }

    void process()
    {
    }

};

class GraphTest : public ::testing::Test {
protected:
    Settings settings;
    NodeFactory factory;

    GraphTest()
        : factory(settings, nullptr)
    {
        settings.set("headless", true);

        std::vector<TagPtr> tags;
        csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor(
                                                     settings,
                                                     "MockupNode", "A mockup node",
                                                     ":/no_icon.png", tags,
                                                     std::bind(&GraphTest::makeMockup)));
        factory.registerNodeType(constructor);
    }

    virtual ~GraphTest() {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp() {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    virtual void TearDown() {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    static NodePtr makeMockup() {
        return NodePtr(new MockupNode);
    }

};

TEST_F(GraphTest, NodeCanBeFound) {
    Graph graph;
    UUID node_id = UUID::make_forced("foobarbaz");
    NodeWorkerPtr node = factory.makeNode("MockupNode", node_id);
    graph.addNode(node);

    NodeWorker* node_found = graph.findNodeWorker(node_id);

    ASSERT_TRUE(node_found != nullptr);
    EXPECT_EQ(node_id, node_found->getUUID());
}

TEST_F(GraphTest, NodeCanBeDeleted) {
    Graph graph;
    UUID node_id = UUID::make_forced("foobarbaz");
    NodeWorkerPtr node = factory.makeNode("MockupNode", node_id);
    graph.addNode(node);

    graph.deleteNode(node_id);

    ASSERT_THROW(graph.findNode(node_id), Graph::NodeNotFoundException);
}

TEST_F(GraphTest, UnknownNodeCannotBeFound) {
    Graph graph;
    UUID node_id = UUID::make_forced("foobarbaz");

    ASSERT_THROW(graph.findNode(node_id), Graph::NodeNotFoundException);
}

TEST_F(GraphTest, nullptr) {
    Graph graph;
    UUID node_id = UUID::make_forced("foobarbaz");

    Node* node_found = graph.findNodeNoThrow(node_id);
    ASSERT_TRUE(node_found == nullptr);
}

}
