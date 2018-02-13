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
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/param/parameter_factory.h>

#include <csapex_testing/stepping_test.h>

#include "gtest/gtest.h"

#include <yaml-cpp/yaml.h>

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

    void setupParameters(Parameterizable& params) override
    {
        params.addHiddenParameter(param::ParameterFactory::declareValue<M>("value", {}));
    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        output = node_modifier.addOutput<M>("output");
    }

    void process() override
    {
        M value = readParameter<M>("value");
//        ainfo << getpid() << " send value: " << value << std::endl;
        msg::publish(output, value);
    }

    bool canProcess() const override
    {
        return Node::canProcess();
    }

    void setValue(M val)
    {
        auto param = getParameter("value");

//        ainfo << "set value: " << val << std::endl;
        param->set(val);
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

    void setupParameters(Parameterizable& params) override
    {
        params.addHiddenParameter(param::ParameterFactory::declareValue<M>("value", {}));
    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        input = node_modifier.addInput<M>("input");
    }

    void process() override
    {
        ASSERT_TRUE(input->hasReceived());
        ASSERT_TRUE(msg::hasMessage(input));

//        ainfo << "RECEIVED: " << msg::getValue<M>(input) << std::endl;

        setParameter("value", msg::getValue<M>(input));
    }

    M getValue() const
    {
        return readParameter<M>("value");
    }


private:
    Input* input;
};

}

class PolymorphicValuesTest : public SteppingTest {
protected:
    NodeFactoryImplementation factory;

    PolymorphicValuesTest() :
        factory(SettingsImplementation::NoSettings, nullptr),
        uuid_provider(std::make_shared<UUIDProvider>())
    {
        std::vector<TagPtr> tags;
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("IntSourceNode",
                                                                                 std::bind(&PolymorphicValuesTest::makeSourceNode<int>)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("IntSinkNode",
                                                                                 std::bind(&PolymorphicValuesTest::makeSinkNode<int>)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("DoubleSourceNode",
                                                                                 std::bind(&PolymorphicValuesTest::makeSourceNode<double>)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("DoubleSinkNode",
                                                                                 std::bind(&PolymorphicValuesTest::makeSinkNode<double>)));
            factory.registerNodeType(constructor);

        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("BoolSourceNode",
                                                                                 std::bind(&PolymorphicValuesTest::makeSourceNode<bool>)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("BoolSinkNode",
                                                                                 std::bind(&PolymorphicValuesTest::makeSinkNode<bool>)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("StringSourceNode",
                                                                                 std::bind(&PolymorphicValuesTest::makeSourceNode<std::string>)));
            factory.registerNodeType(constructor);
        }
        {
            csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor("StringSinkNode",
                                                                                 std::bind(&PolymorphicValuesTest::makeSinkNode<std::string>)));
            factory.registerNodeType(constructor);
        }
    }

    virtual ~PolymorphicValuesTest() {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    template <typename T>
    static NodePtr makeSourceNode() {
        return NodePtr(new MSourceNode<T>);
    }
    template <typename T>
    static NodePtr makeSinkNode() {
        return NodePtr(new MSinkNode<T>);
    }
    UUIDProviderPtr uuid_provider;
};

TEST_F(PolymorphicValuesTest, IntCanBeConnectedToInt) {
    OutputPtr o = std::make_shared<StaticOutput>(uuid_provider->makeUUID("o1"));
    InputPtr i = std::make_shared<Input>(uuid_provider->makeUUID("in"));

    o->setType(std::make_shared<connection_types::GenericValueMessage<int>>());
    i->setType(std::make_shared<connection_types::GenericValueMessage<int>>());


    // is compatible
    ASSERT_TRUE(Connection::isCompatibleWith(o.get(), i.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(i.get(), o.get()));
    // can be connected
    ASSERT_TRUE(Connection::canBeConnectedTo(o.get(), i.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(i.get(), o.get()));

    ConnectionPtr c = DirectConnection::connect(o, i);
    ASSERT_NE(nullptr, c);
}


TEST_F(PolymorphicValuesTest, DoubleCanBeConnectedToDouble) {
    OutputPtr o = std::make_shared<StaticOutput>(uuid_provider->makeUUID("o1"));
    InputPtr i = std::make_shared<Input>(uuid_provider->makeUUID("in"));

    o->setType(std::make_shared<connection_types::GenericValueMessage<double>>());
    i->setType(std::make_shared<connection_types::GenericValueMessage<double>>());


    // is compatible
    ASSERT_TRUE(Connection::isCompatibleWith(o.get(), i.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(i.get(), o.get()));
    // can be connected
    ASSERT_TRUE(Connection::canBeConnectedTo(o.get(), i.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(i.get(), o.get()));

    ConnectionPtr c = DirectConnection::connect(o, i);
    ASSERT_NE(nullptr, c);
}

TEST_F(PolymorphicValuesTest, ExactValueDoesNotUseMessageConversion) {
    InputPtr i = std::make_shared<Input>(uuid_provider->makeUUID("in"));

    i->setType(std::make_shared<connection_types::GenericValueMessage<int>>());
    i->setToken(std::make_shared<Token>(connection_types::makeEmptyMessage<connection_types::GenericValueMessage<int>>()));
    ASSERT_TRUE(msg::isValue<int>(i.get()));
    ASSERT_TRUE(msg::isValue<double>(i.get()));
    ASSERT_TRUE(msg::isExactValue<int>(i.get()));
    ASSERT_FALSE(msg::isExactValue<double>(i.get()));
    i->setType(std::make_shared<connection_types::GenericValueMessage<double>>());
    i->setToken(std::make_shared<Token>(connection_types::makeEmptyMessage<connection_types::GenericValueMessage<double>>()));
    ASSERT_TRUE(msg::isValue<int>(i.get()));
    ASSERT_TRUE(msg::isValue<double>(i.get()));
    ASSERT_FALSE(msg::isExactValue<int>(i.get()));
    ASSERT_TRUE(msg::isExactValue<double>(i.get()));

}
TEST_F(PolymorphicValuesTest, IntCanBeConnectedToDouble) {
    OutputPtr o = std::make_shared<StaticOutput>(uuid_provider->makeUUID("o1"));
    InputPtr i = std::make_shared<Input>(uuid_provider->makeUUID("in"));

    o->setType(std::make_shared<connection_types::GenericValueMessage<int>>());
    i->setType(std::make_shared<connection_types::GenericValueMessage<double>>());

    // is compatible
    ASSERT_TRUE(Connection::isCompatibleWith(o.get(), i.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(i.get(), o.get()));
    // can be connected
    ASSERT_TRUE(Connection::canBeConnectedTo(o.get(), i.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(i.get(), o.get()));

    ConnectionPtr c = DirectConnection::connect(o, i);
    ASSERT_NE(nullptr, c);
}

TEST_F(PolymorphicValuesTest, DoubleCanBeConnectedToInt) {
    OutputPtr o = std::make_shared<StaticOutput>(uuid_provider->makeUUID("o1"));
    InputPtr i = std::make_shared<Input>(uuid_provider->makeUUID("in"));

    o->setType(std::make_shared<connection_types::GenericValueMessage<double>>());
    i->setType(std::make_shared<connection_types::GenericValueMessage<int>>());

    // is compatible
    ASSERT_TRUE(Connection::isCompatibleWith(o.get(), i.get()));
    ASSERT_TRUE(Connection::isCompatibleWith(i.get(), o.get()));
    // can be connected
    ASSERT_TRUE(Connection::canBeConnectedTo(o.get(), i.get()));
    ASSERT_TRUE(Connection::canBeConnectedTo(i.get(), o.get()));

    ConnectionPtr c = DirectConnection::connect(o, i);
    ASSERT_NE(nullptr, c);
}

TEST_F(PolymorphicValuesTest, IntCanBeConnectedToIntViaNodes) {
    GraphFacadeImplementation graph_facade(executor, graph, graph_node);

    UUID node1_id = UUIDProvider::makeUUID_without_parent("node1");
    NodeFacadeImplementationPtr node_a = factory.makeNode("IntSourceNode", node1_id, graph);
    graph_facade.addNode(node_a);

    UUID node2_id = UUIDProvider::makeUUID_without_parent("node2");
    NodeFacadeImplementationPtr node_b = factory.makeNode("IntSinkNode", node2_id, graph);
    graph_facade.addNode(node_b);

    graph_facade.connect(graph->makeTypedUUID_forced(node1_id, "out", 0),
                         graph->makeTypedUUID_forced(node2_id, "in", 0));


    executor.start();

    auto sender = dynamic_cast<MSourceNode<int>*>(node_a->getNode().get());
    auto receiver = dynamic_cast<MSinkNode<int>*>(node_b->getNode().get());

    ASSERT_NE(nullptr, sender);
    ASSERT_NE(nullptr, receiver);

    for(int iter = 0; iter < 23; ++iter) {
        sender->setValue(iter);
        ASSERT_NO_FATAL_FAILURE(step());
        ASSERT_EQ(iter, receiver->getValue());
    }
}

TEST_F(PolymorphicValuesTest, DoubleCanBeConnectedToDoubleViaNodes) {
    GraphFacadeImplementation graph_facade(executor, graph, graph_node);

    UUID node1_id = UUIDProvider::makeUUID_without_parent("node1");
    NodeFacadeImplementationPtr node_a = factory.makeNode("DoubleSourceNode", node1_id, graph);
    graph_facade.addNode(node_a);

    UUID node2_id = UUIDProvider::makeUUID_without_parent("node2");
    NodeFacadeImplementationPtr node_b = factory.makeNode("DoubleSinkNode", node2_id, graph);
    graph_facade.addNode(node_b);

    graph_facade.connect(graph->makeTypedUUID_forced(node1_id, "out", 0),
                         graph->makeTypedUUID_forced(node2_id, "in", 0));


    executor.start();

    auto sender = dynamic_cast<MSourceNode<double>*>(node_a->getNode().get());
    auto receiver = dynamic_cast<MSinkNode<double>*>(node_b->getNode().get());

    ASSERT_NE(nullptr, sender);
    ASSERT_NE(nullptr, receiver);

    for(int iter = 0; iter < 23; ++iter) {
        sender->setValue(iter + 0.5);
        ASSERT_NO_FATAL_FAILURE(step());
        ASSERT_EQ(iter + 0.5, receiver->getValue());
    }
}

TEST_F(PolymorphicValuesTest, DoubleCanBeConnectedToIntViaNodes) {
    GraphFacadeImplementation graph_facade(executor, graph, graph_node);

    UUID node1_id = UUIDProvider::makeUUID_without_parent("node1");
    NodeFacadeImplementationPtr node_a = factory.makeNode("DoubleSourceNode", node1_id, graph);
    graph_facade.addNode(node_a);

    UUID node2_id = UUIDProvider::makeUUID_without_parent("node2");
    NodeFacadeImplementationPtr node_b = factory.makeNode("IntSinkNode", node2_id, graph);
    graph_facade.addNode(node_b);

    graph_facade.connect(graph->makeTypedUUID_forced(node1_id, "out", 0),
                         graph->makeTypedUUID_forced(node2_id, "in", 0));


    executor.start();

    auto sender = dynamic_cast<MSourceNode<double>*>(node_a->getNode().get());
    auto receiver = dynamic_cast<MSinkNode<int>*>(node_b->getNode().get());

    ASSERT_NE(nullptr, sender);
    ASSERT_NE(nullptr, receiver);

    for(int iter = 0; iter < 23; ++iter) {
        sender->setValue(iter + 0.25);
        ASSERT_NO_FATAL_FAILURE(step());
        ASSERT_EQ(iter, receiver->getValue());
    }
}


TEST_F(PolymorphicValuesTest, IntCanBeConnectedToDoubleViaNodes) {
    GraphFacadeImplementation graph_facade(executor, graph, graph_node);

    UUID node1_id = UUIDProvider::makeUUID_without_parent("node1");
    NodeFacadeImplementationPtr node_a = factory.makeNode("IntSourceNode", node1_id, graph);
    graph_facade.addNode(node_a);

    UUID node2_id = UUIDProvider::makeUUID_without_parent("node2");
    NodeFacadeImplementationPtr node_b = factory.makeNode("DoubleSinkNode", node2_id, graph);
    graph_facade.addNode(node_b);

    graph_facade.connect(graph->makeTypedUUID_forced(node1_id, "out", 0),
                         graph->makeTypedUUID_forced(node2_id, "in", 0));


    executor.start();

    auto sender = dynamic_cast<MSourceNode<int>*>(node_a->getNode().get());
    auto receiver = dynamic_cast<MSinkNode<double>*>(node_b->getNode().get());

    ASSERT_NE(nullptr, sender);
    ASSERT_NE(nullptr, receiver);

    for(int iter = 0; iter < 23; ++iter) {
        sender->setValue(iter);
        ASSERT_NO_FATAL_FAILURE(step());
        ASSERT_NEAR(iter, receiver->getValue(), 0.001);
    }
}


TEST_F(PolymorphicValuesTest, DoubleCanBeConnectedToBoolViaNodes) {
    GraphFacadeImplementation graph_facade(executor, graph, graph_node);

    UUID node1_id = UUIDProvider::makeUUID_without_parent("node1");
    NodeFacadeImplementationPtr node_a = factory.makeNode("DoubleSourceNode", node1_id, graph);
    graph_facade.addNode(node_a);

    UUID node2_id = UUIDProvider::makeUUID_without_parent("node2");
    NodeFacadeImplementationPtr node_b = factory.makeNode("BoolSinkNode", node2_id, graph);
    graph_facade.addNode(node_b);

    graph_facade.connect(graph->makeTypedUUID_forced(node1_id, "out", 0),
                         graph->makeTypedUUID_forced(node2_id, "in", 0));


    executor.start();

    auto sender = dynamic_cast<MSourceNode<double>*>(node_a->getNode().get());
    auto receiver = dynamic_cast<MSinkNode<bool>*>(node_b->getNode().get());

    ASSERT_NE(nullptr, sender);
    ASSERT_NE(nullptr, receiver);

    for(int iter = 0; iter < 23; ++iter) {
        sender->setValue(iter);
        ASSERT_NO_FATAL_FAILURE(step());
        ASSERT_EQ(iter != 0, receiver->getValue());
    }
}


TEST_F(PolymorphicValuesTest, BoolCanBeConnectedToDoubleViaNodes) {
    GraphFacadeImplementation graph_facade(executor, graph, graph_node);

    UUID node1_id = UUIDProvider::makeUUID_without_parent("node1");
    NodeFacadeImplementationPtr node_a = factory.makeNode("BoolSourceNode", node1_id, graph);
    graph_facade.addNode(node_a);

    UUID node2_id = UUIDProvider::makeUUID_without_parent("node2");
    NodeFacadeImplementationPtr node_b = factory.makeNode("DoubleSinkNode", node2_id, graph);
    graph_facade.addNode(node_b);

    graph_facade.connect(graph->makeTypedUUID_forced(node1_id, "out", 0),
                         graph->makeTypedUUID_forced(node2_id, "in", 0));


    executor.start();

    auto sender = dynamic_cast<MSourceNode<bool>*>(node_a->getNode().get());
    auto receiver = dynamic_cast<MSinkNode<double>*>(node_b->getNode().get());

    ASSERT_NE(nullptr, sender);
    ASSERT_NE(nullptr, receiver);

    for(int iter = 0; iter < 23; ++iter) {
        sender->setValue((iter % 2) == 0);
        ASSERT_NO_FATAL_FAILURE(step());

        if((iter % 2) == 0) {
            ASSERT_NEAR(1.0, receiver->getValue(), 0.0001);

        } else {
            ASSERT_NEAR(0.0, receiver->getValue(), 0.0001);
        }
    }
}



TEST_F(PolymorphicValuesTest, BoolCanBeConnectedToStringViaNodes) {
    GraphFacadeImplementation graph_facade(executor, graph, graph_node);

    UUID node1_id = UUIDProvider::makeUUID_without_parent("node1");
    NodeFacadeImplementationPtr node_a = factory.makeNode("BoolSourceNode", node1_id, graph);
    graph_facade.addNode(node_a);

    UUID node2_id = UUIDProvider::makeUUID_without_parent("node2");
    NodeFacadeImplementationPtr node_b = factory.makeNode("StringSinkNode", node2_id, graph);
    graph_facade.addNode(node_b);

    graph_facade.connect(graph->makeTypedUUID_forced(node1_id, "out", 0),
                         graph->makeTypedUUID_forced(node2_id, "in", 0));


    executor.start();

    auto sender = dynamic_cast<MSourceNode<bool>*>(node_a->getNode().get());
    auto receiver = dynamic_cast<MSinkNode<std::string>*>(node_b->getNode().get());

    ASSERT_NE(nullptr, sender);
    ASSERT_NE(nullptr, receiver);

    for(int iter = 0; iter < 23; ++iter) {
        sender->setValue((iter % 2) == 0);
        ASSERT_NO_FATAL_FAILURE(step());

        if((iter % 2) == 0) {
            ASSERT_EQ("true", receiver->getValue());

        } else {
            ASSERT_EQ("false", receiver->getValue());
        }
    }
}
}
