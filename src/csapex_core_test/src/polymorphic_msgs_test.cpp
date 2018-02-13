#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/core/settings/settings_impl.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/input.h>
#include <csapex/msg/message_template.hpp>
#include <csapex/msg/direct_connection.h>
#include <csapex/msg/static_output.h>
#include <csapex/msg/input.h>
#include <csapex/utility/register_msg.h>

#include <csapex_testing/stepping_test.h>

#include "gtest/gtest.h"

#include <yaml-cpp/yaml.h>

namespace csapex {


struct Base {};
struct Child : public Base {};

namespace connection_types {

struct BaseMessage : public MessageTemplate<Base, BaseMessage>
{

};

template <>
struct type<BaseMessage> {
    static std::string name() {
        return "Base";
    }
};




struct ChildMessage : public MessageTemplate<Child, ChildMessage>
{
public:
    int child_value;
};

template <>
struct type<ChildMessage> {
    static std::string name() {
        return "Child";
    }
};


}
}

CSAPEX_REGISTER_MESSAGE(BaseMessage)
CSAPEX_REGISTER_MESSAGE(ChildMessage)


/// YAML
namespace YAML {
template<>
struct CSAPEX_CORE_EXPORT convert<csapex::connection_types::BaseMessage> {
  static Node encode(const csapex::connection_types::BaseMessage& rhs){
      return {};
  }
  static bool decode(const Node& node, csapex::connection_types::BaseMessage& rhs){
      return true;
  }
};
template<>
struct CSAPEX_CORE_EXPORT convert<csapex::connection_types::ChildMessage> {
  static Node encode(const csapex::connection_types::ChildMessage& rhs){
      return {};
  }
  static bool decode(const Node& node, csapex::connection_types::ChildMessage& rhs) {
      return true;
  }
};
}
namespace csapex
{

namespace {

template <typename M>
class MSourceNode : public Node
{
public:
    MSourceNode()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        output = node_modifier.addOutput<M>("output");
    }

    void process() override
    {
        auto m = std::make_shared<M>();
        msg::publish(output, m);
    }

private:
    Output* output;
};

template <typename M>
class MSinkNode : public Node
{
public:
    MSinkNode()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input = node_modifier.addInput<M>("input");
    }

    void process() override
    {
        ASSERT_TRUE(msg::hasMessage(input));

        auto msg = msg::getMessage(input);
        ASSERT_NE(nullptr, msg);

        ASSERT_NE(nullptr, csapex::msg::message_cast<M const> (msg));

        auto base_msg = msg::getMessage<M>(input);
        ASSERT_NE(nullptr, base_msg);
    }

private:
    Input* input;
};

}

class PolymorphicMsgsTest : public SteppingTest {
protected:
    NodeFactoryImplementation factory;

    PolymorphicMsgsTest() :
        factory(SettingsImplementation::NoSettings, nullptr),
        uuid_provider(std::make_shared<UUIDProvider>())
    {
        std::vector<TagPtr> tags;
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("BaseSourceNode",
                                                                                 std::bind(&PolymorphicMsgsTest::makeBaseSource)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("BaseSinkNode",
                                                                                 std::bind(&PolymorphicMsgsTest::makeBaseSink)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("ChildSourceNode",
                                                                                 std::bind(&PolymorphicMsgsTest::makeChildSource)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("ChildSinkNode",
                                                                                 std::bind(&PolymorphicMsgsTest::makeChildSink)));
            factory.registerNodeType(constructor);
        }
    }

    virtual ~PolymorphicMsgsTest() {
        // You can do clean-up work that doesn't throw exceptions here.
    }


    static NodePtr makeBaseSource() {
        return NodePtr(new MSourceNode<connection_types::BaseMessage>);
    }
    static NodePtr makeBaseSink() {
        return NodePtr(new MSinkNode<connection_types::BaseMessage>);
    }
    static NodePtr makeChildSource() {
        return NodePtr(new MSourceNode<connection_types::ChildMessage>);
    }
    static NodePtr makeChildSink() {
        return NodePtr(new MSinkNode<connection_types::ChildMessage>);
    }

    UUIDProviderPtr uuid_provider;
};

TEST_F(PolymorphicMsgsTest, BaseCanBeConnectedToBase) {
    OutputPtr o = std::make_shared<StaticOutput>(uuid_provider->makeUUID("o1"));
    InputPtr i = std::make_shared<Input>(uuid_provider->makeUUID("in"));

    o->setType(std::make_shared<connection_types::BaseMessage>());
    i->setType(std::make_shared<connection_types::BaseMessage>());


    // is compatible
    ASSERT_TRUE(Connection::isCompatibleWith(o.get(), i.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(i.get(), o.get()));
    // can be connected
    ASSERT_TRUE(Connection::canBeConnectedTo(o.get(), i.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(i.get(), o.get()));

    ConnectionPtr c = DirectConnection::connect(o, i);
    ASSERT_NE(nullptr, c);
}


TEST_F(PolymorphicMsgsTest, ChildCanBeConnectedToChild) {
    OutputPtr o = std::make_shared<StaticOutput>(uuid_provider->makeUUID("o1"));
    InputPtr i = std::make_shared<Input>(uuid_provider->makeUUID("in"));

    o->setType(std::make_shared<connection_types::ChildMessage>());
    i->setType(std::make_shared<connection_types::ChildMessage>());


    // is compatible
    ASSERT_TRUE(Connection::isCompatibleWith(o.get(), i.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(i.get(), o.get()));
    // can be connected
    ASSERT_TRUE(Connection::canBeConnectedTo(o.get(), i.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(i.get(), o.get()));

    ConnectionPtr c = DirectConnection::connect(o, i);
    ASSERT_NE(nullptr, c);
}

TEST_F(PolymorphicMsgsTest, BaseCannotBeConnectedToChild) {
    OutputPtr o = std::make_shared<StaticOutput>(uuid_provider->makeUUID("o1"));
    InputPtr i = std::make_shared<Input>(uuid_provider->makeUUID("in"));

    o->setType(std::make_shared<connection_types::BaseMessage>());
    i->setType(std::make_shared<connection_types::ChildMessage>());

    // is compatible
    ASSERT_FALSE(Connection::isCompatibleWith(o.get(), i.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(i.get(), o.get()));
    // cannot be connected in both directions
    ASSERT_FALSE(Connection::canBeConnectedTo(o.get(), i.get())); // would involve down-cast
    ASSERT_TRUE(Connection::canBeConnectedTo(i.get(), o.get())); // needs upcast (ok)

    ASSERT_EQ(nullptr, DirectConnection::connect(o, i));
}

TEST_F(PolymorphicMsgsTest, BaseCanBeConnectedToBaseViaNodes) {
    GraphFacadeImplementation graph_facade(executor, graph, graph_node);

    UUID node1_id = UUIDProvider::makeUUID_without_parent("node1");
    NodeFacadeImplementationPtr node_a = factory.makeNode("BaseSourceNode", node1_id, graph);
    graph_facade.addNode(node_a);

    UUID node2_id = UUIDProvider::makeUUID_without_parent("node2");
    NodeFacadeImplementationPtr node_b = factory.makeNode("BaseSinkNode", node2_id, graph);
    graph_facade.addNode(node_b);

    graph_facade.connect(graph->makeTypedUUID_forced(node1_id, "out", 0),
                         graph->makeTypedUUID_forced(node2_id, "in", 0));


    executor.start();

    for(int iter = 0; iter < 23; ++iter) {
        ASSERT_NO_FATAL_FAILURE(step());
    }
}


TEST_F(PolymorphicMsgsTest, ChildCanBeConnectedToChildViaNodes) {
    GraphFacadeImplementation graph_facade(executor, graph, graph_node);

    UUID node1_id = UUIDProvider::makeUUID_without_parent("node1");
    NodeFacadeImplementationPtr node_a = factory.makeNode("ChildSourceNode", node1_id, graph);
    graph_facade.addNode(node_a);

    UUID node2_id = UUIDProvider::makeUUID_without_parent("node2");
    NodeFacadeImplementationPtr node_b = factory.makeNode("ChildSinkNode", node2_id, graph);
    graph_facade.addNode(node_b);

    graph_facade.connect(graph->makeTypedUUID_forced(node1_id, "out", 0),
                         graph->makeTypedUUID_forced(node2_id, "in", 0));

    executor.start();

    for(int iter = 0; iter < 23; ++iter) {
        ASSERT_NO_FATAL_FAILURE(step());
    }
}



TEST_F(PolymorphicMsgsTest, ChildCanBeConnectedToBaseViaNodes) {
    GraphFacadeImplementation graph_facade(executor, graph, graph_node);

    UUID node1_id = UUIDProvider::makeUUID_without_parent("node1");
    NodeFacadeImplementationPtr node_a = factory.makeNode("ChildSourceNode", node1_id, graph);
    graph_facade.addNode(node_a);

    UUID node2_id = UUIDProvider::makeUUID_without_parent("node2");
    NodeFacadeImplementationPtr node_b = factory.makeNode("BaseSinkNode", node2_id, graph);
    graph_facade.addNode(node_b);

    graph_facade.connect(graph->makeTypedUUID_forced(node1_id, "out", 0),
                         graph->makeTypedUUID_forced(node2_id, "in", 0));

    executor.start();

    for(int iter = 0; iter < 23; ++iter) {
        ASSERT_NO_FATAL_FAILURE(step());
    }
}


TEST_F(PolymorphicMsgsTest, BaseCannotBeConnectedToChildViaNodes) {
    GraphFacadeImplementation graph_facade(executor, graph, graph_node);

    UUID node1_id = UUIDProvider::makeUUID_without_parent("node1");
    NodeFacadeImplementationPtr node_a = factory.makeNode("BaseSourceNode", node1_id, graph);
    graph_facade.addNode(node_a);

    UUID node2_id = UUIDProvider::makeUUID_without_parent("node2");
    NodeFacadeImplementationPtr node_b = factory.makeNode("ChildSinkNode", node2_id, graph);
    graph_facade.addNode(node_b);

    ASSERT_THROW(graph_facade.connect(graph->makeTypedUUID_forced(node1_id, "out", 0),
                                      graph->makeTypedUUID_forced(node2_id, "in", 0)),
                 csapex::HardAssertionFailure);
}


}
