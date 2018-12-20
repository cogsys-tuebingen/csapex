#include <csapex_testing/stepping_test.h>
#include <csapex/core/settings/settings_impl.h>

#include <csapex/core/csapex_core.h>
#include <csapex/command/command_factory.h>
#include <csapex/command/add_node.h>
#include <csapex/command/delete_node.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex_testing/mockup_nodes.h>

namespace csapex
{
namespace detail
{
template <typename T>
NodePtr makeNode()
{
    return NodePtr(new T());
}
}  // namespace detail
}  // namespace csapex

namespace csapex
{
class CommandTest : public CsApexTestCase
{
};

TEST_F(CommandTest, AddNodeAndUndoWorks)
{
    ExceptionHandler eh(false);
    SettingsImplementation settings;

    std::string path_to_bin("");
    settings.set("path_to_bin", path_to_bin);
    settings.set("use_boot_plugins", false);

    CsApexCore core(settings, eh);

    NodeFactoryImplementation& factory = *core.getNodeFactory();
    GraphFacadeImplementationPtr graph = core.getRoot();

    factory.registerNodeType(std::make_shared<NodeConstructor>("MockupSource", std::bind(&detail::makeNode<MockupSource>)));

    CommandFactory cmd_factory(graph.get());

    CommandDispatcher& dispatcher = *core.getCommandDispatcher();

    // ADD
    auto node_uuid = graph->generateUUID("MockupSource");
    CommandPtr add_node = std::make_shared<command::AddNode>(graph->getAbsoluteUUID(), "MockupSource", Point{ 50.0, 50.0 }, node_uuid, NodeStatePtr());
    ASSERT_NE(nullptr, add_node);

    EXPECT_EQ(0, graph->countNodes());

    EXPECT_FALSE(dispatcher.isDirty());
    EXPECT_FALSE(dispatcher.canUndo());
    ASSERT_TRUE(dispatcher.execute(add_node));
    EXPECT_TRUE(dispatcher.isDirty());
    EXPECT_TRUE(dispatcher.canUndo());

    ASSERT_EQ(1, graph->countNodes());

    // UNDO
    EXPECT_TRUE(dispatcher.isDirty());
    ASSERT_TRUE(dispatcher.canUndo());
    ASSERT_NO_THROW(dispatcher.undo());
    EXPECT_FALSE(dispatcher.isDirty());
    EXPECT_FALSE(dispatcher.canUndo());

    EXPECT_EQ(0, graph->countNodes());
}

TEST_F(CommandTest, AddNodeAndDeleteWorks)
{
    ExceptionHandler eh(false);
    SettingsImplementation settings;

    std::string path_to_bin("");
    settings.set("path_to_bin", path_to_bin);
    settings.set("use_boot_plugins", false);

    CsApexCore core(settings, eh);

    NodeFactoryImplementation& factory = *core.getNodeFactory();
    GraphFacadeImplementationPtr graph = core.getRoot();

    factory.registerNodeType(std::make_shared<NodeConstructor>("MockupSource", std::bind(&detail::makeNode<MockupSource>)));

    CommandFactory cmd_factory(graph.get());

    CommandDispatcher& dispatcher = *core.getCommandDispatcher();

    // ADD
    auto node_uuid = graph->generateUUID("MockupSource");
    CommandPtr add_node = std::make_shared<command::AddNode>(graph->getAbsoluteUUID(), "MockupSource", Point{ 50.0, 50.0 }, node_uuid, NodeStatePtr());
    ASSERT_NE(nullptr, add_node);

    EXPECT_EQ(0, graph->countNodes());

    EXPECT_FALSE(dispatcher.isDirty());
    EXPECT_FALSE(dispatcher.canUndo());
    ASSERT_TRUE(dispatcher.execute(add_node));
    EXPECT_TRUE(dispatcher.isDirty());
    EXPECT_TRUE(dispatcher.canUndo());

    ASSERT_EQ(1, graph->countNodes());

    // DELETE
    CommandPtr delete_node = std::make_shared<command::DeleteNode>(graph->getAbsoluteUUID(), node_uuid);
    ASSERT_NE(nullptr, delete_node);

    EXPECT_TRUE(dispatcher.isDirty());
    ASSERT_TRUE(dispatcher.canUndo());
    ASSERT_TRUE(dispatcher.execute(delete_node));
    EXPECT_TRUE(dispatcher.isDirty());
    ASSERT_TRUE(dispatcher.canUndo());

    EXPECT_EQ(0, graph->countNodes());

    // UNDO DeleteNode
    EXPECT_TRUE(dispatcher.isDirty());
    ASSERT_TRUE(dispatcher.canUndo());
    ASSERT_NO_THROW(dispatcher.undo());
    EXPECT_TRUE(dispatcher.isDirty());
    EXPECT_TRUE(dispatcher.canUndo());

    EXPECT_EQ(1, graph->countNodes());

    // UNDO AddNode
    EXPECT_TRUE(dispatcher.isDirty());
    ASSERT_TRUE(dispatcher.canUndo());
    ASSERT_NO_THROW(dispatcher.undo());
    EXPECT_FALSE(dispatcher.isDirty());
    EXPECT_FALSE(dispatcher.canUndo());

    EXPECT_EQ(0, graph->countNodes());
}

TEST_F(CommandTest, DeleteAllWorks)
{
    ExceptionHandler eh(false);
    SettingsImplementation settings;

    std::string path_to_bin("");
    settings.set("path_to_bin", path_to_bin);
    settings.set("use_boot_plugins", false);

    CsApexCore core(settings, eh);

    NodeFactoryImplementation& factory = *core.getNodeFactory();
    GraphFacadeImplementationPtr graph = core.getRoot();

    factory.registerNodeType(std::make_shared<NodeConstructor>("MockupSource", std::bind(&detail::makeNode<MockupSource>)));

    CommandFactory cmd_factory(graph.get());

    CommandDispatcher& dispatcher = *core.getCommandDispatcher();

    // ADD
    auto node_uuid = graph->generateUUID("MockupSource");
    CommandPtr add_node = std::make_shared<command::AddNode>(graph->getAbsoluteUUID(), "MockupSource", Point{ 50.0, 50.0 }, node_uuid, NodeStatePtr());
    ASSERT_NE(nullptr, add_node);

    EXPECT_EQ(0, graph->countNodes());

    EXPECT_FALSE(dispatcher.isDirty());
    EXPECT_FALSE(dispatcher.canUndo());
    ASSERT_TRUE(dispatcher.execute(add_node));
    EXPECT_TRUE(dispatcher.isDirty());
    EXPECT_TRUE(dispatcher.canUndo());

    ASSERT_EQ(1, graph->countNodes());

    // DELETE

    int order = 0;
    int state_changed_called = -1;
    graph->state_changed.connect([&]() { state_changed_called = order++; });
    int node_removed_called = -1;
    graph->node_facade_removed.connect([&]() { node_removed_called = order++; });

    CommandPtr delete_all = cmd_factory.deleteAllNodes({ node_uuid });
    ASSERT_NE(nullptr, delete_all);

    EXPECT_TRUE(dispatcher.isDirty());
    ASSERT_TRUE(dispatcher.canUndo());
    ASSERT_TRUE(dispatcher.execute(delete_all));
    EXPECT_TRUE(dispatcher.isDirty());
    ASSERT_TRUE(dispatcher.canUndo());

    EXPECT_EQ(0, node_removed_called);
    EXPECT_EQ(1, state_changed_called);

    EXPECT_EQ(0, graph->countNodes());
}
}  // namespace csapex
