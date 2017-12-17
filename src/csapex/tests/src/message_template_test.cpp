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

#include "stepping_test.h"

#include "gtest/gtest.h"

#include <yaml-cpp/yaml.h>

namespace csapex {

struct Foo
{
    Foo()
        : value(-1)
    {}
    Foo(int v)
        : value(v)
    {}

    int value;
};

namespace connection_types {

struct VectorMessage : public MessageTemplate<std::vector<Foo>, VectorMessage>
{

};

template <>
struct type<VectorMessage> {
    static std::string name() {
        return "TestVector";
    }
};


}
}

CSAPEX_REGISTER_MESSAGE(VectorMessage)


/// YAML
namespace YAML {
template<>
struct CSAPEX_EXPORT convert<csapex::connection_types::VectorMessage> {
  static Node encode(const csapex::connection_types::VectorMessage& rhs){
      return {};
  }
  static bool decode(const Node& node, csapex::connection_types::VectorMessage& rhs){
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
        m->value.resize(23, Foo(42));
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

        auto vector_msg = msg::getMessage<M>(input);
        ASSERT_NE(nullptr, vector_msg);

        ASSERT_EQ(23, vector_msg->value.size());
    }

private:
    Input* input;
};

}

class MessageTemplateTest : public SteppingTest {
protected:
    NodeFactoryImplementation factory;

    MessageTemplateTest() :
        factory(SettingsImplementation::NoSettings, nullptr),
        uuid_provider(std::make_shared<UUIDProvider>())
    {
        std::vector<TagPtr> tags;
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("VectorSourceNode",
                                                                                 std::bind(&MessageTemplateTest::makeVectorSource)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("VectorSinkNode",
                                                                                 std::bind(&MessageTemplateTest::makeVectorSink)));
            factory.registerNodeType(constructor);
        }
    }

    virtual ~MessageTemplateTest() {
        // You can do clean-up work that doesn't throw exceptions here.
    }


    static NodePtr makeVectorSource() {
        return NodePtr(new MSourceNode<connection_types::VectorMessage>);
    }
    static NodePtr makeVectorSink() {
        return NodePtr(new MSinkNode<connection_types::VectorMessage>);
    }

    UUIDProviderPtr uuid_provider;
};

TEST_F(MessageTemplateTest, VectorCanBeConnectedToVector) {
    OutputPtr o = std::make_shared<StaticOutput>(uuid_provider->makeUUID("o1"));
    InputPtr i = std::make_shared<Input>(uuid_provider->makeUUID("in"));

    o->setType(std::make_shared<connection_types::VectorMessage>());
    i->setType(std::make_shared<connection_types::VectorMessage>());


    // is compatible
    ASSERT_TRUE(Connection::isCompatibleWith(o.get(), i.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(i.get(), o.get()));
    // can be connected
    ASSERT_TRUE(Connection::canBeConnectedTo(o.get(), i.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(i.get(), o.get()));

    ConnectionPtr c = DirectConnection::connect(o, i);
    ASSERT_NE(nullptr, c);
}



TEST_F(MessageTemplateTest, ChildCanBeConnectedToVectorViaNodes) {
    GraphFacadeImplementation graph_facade(executor, graph, graph_node);

    UUID node1_id = UUIDProvider::makeUUID_without_parent("source");
    NodeFacadeImplementationPtr node_a = factory.makeNode("VectorSourceNode", node1_id, graph);
    graph_facade.addNode(node_a);

    const int sinks = 50;
    for (int i = 0; i < sinks; ++i) {
        UUID nodei_id = UUIDProvider::makeUUID_without_parent(std::string("sink_") + std::to_string(i));
        NodeFacadeImplementationPtr node_b = factory.makeNode("VectorSinkNode", nodei_id, graph);
        graph_facade.addNode(node_b);

        graph_facade.connect(graph->makeTypedUUID_forced(node1_id, "out", 0),
                             graph->makeTypedUUID_forced(nodei_id, "in", 0));
    }
    executor.start();

    for(int iter = 0; iter < 23; ++iter) {
        ASSERT_NO_FATAL_FAILURE(step());
    }
}

}
