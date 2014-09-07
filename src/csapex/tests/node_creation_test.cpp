#include <csapex/model/node.h>
#include <csapex/model/node_factory.h>
#include <csapex/msg/message.h>
#include <csapex/core/settings.h>
#include <csapex/factory/generic_node_factory.hpp>
#include <csapex/utility/register_msg.h>
//#include <csapex/msg/message_traits.h>
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


/*
 * Mockup Message
 */

struct MockupMessagePayload
{
    float value;
};

struct MockupMessage : public connection_types::Message
{
    typedef boost::shared_ptr<MockupMessage> Ptr;

    MockupMessage()
        : Message("MockupMessage", "/")
    {
    }

    ConnectionType::Ptr clone() {
        throw std::runtime_error ("MOCKUP: not implemented");
    }

    ConnectionType::Ptr toType() {
        throw std::runtime_error ("MOCKUP: not implemented");
    }

    static ConnectionType::Ptr make(){
        return Ptr(new MockupMessage);
    }


    void writeYaml(YAML::Emitter &) const
    {
        throw std::runtime_error ("MOCKUP: not implemented");
    }

    void readYaml(const YAML::Node &)
    {
        throw std::runtime_error ("MOCKUP: not implemented");
    }

    MockupMessagePayload value;
};

/// TRAITS
namespace connection_types {
template <>
struct type<MockupMessage> {
    static std::string name() {
        return "Mockup";
    }
};
}
}


/// YAML
namespace YAML {
template<>
struct convert<csapex::MockupMessage> {
  static Node encode(const csapex::MockupMessage&){return Node();}
  static bool decode(const Node&, csapex::MockupMessage&){return true;}
};
}

CSAPEX_REGISTER_MESSAGE(csapex::MockupMessage)

namespace csapex {

void functionToBeWrappedIntoANode(const MockupMessage& input, const MockupMessage& input2, int parameter, MockupMessage& output)
{
    output.value.value = input.value.value + input2.value.value + parameter;
}


class NodeCreationTest : public ::testing::Test {
protected:
    Settings settings;
    NodeFactory factory;

    NodeCreationTest()
        : factory(settings)
    {
        settings.set("headless", true);

        std::vector<TagPtr> tags;
        csapex::NodeConstructor::Ptr mockup_constructor(new csapex::NodeConstructor(
                                                            settings,
                                                            "MockupNode", "A mockup node",
                                                            ":/no_icon.png", tags,
                                                            boost::bind(&NodeCreationTest::makeMockup)));
        factory.registerNodeType(mockup_constructor);

        factory.registerNodeType(GenericNodeFactory::createConstructorFromFunction(functionToBeWrappedIntoANode,
                                                                                    "WrappedFunctionNode",
                                                                                    "functionToBeWrappedIntoANode",
                                                                                    settings));
    }

    virtual ~NodeCreationTest() {
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

TEST_F(NodeCreationTest, NodeCanBeMadeInAFactory) {
    UUID node_id = UUID::make_forced("foobarbaz");
    NodePtr node = factory.makeNode("MockupNode", node_id);

    ASSERT_TRUE(node != NULL);
    EXPECT_EQ(node_id, node->getUUID());
}

TEST_F(NodeCreationTest, GenericNodeCanBeMadeInAFactory) {
    UUID node_id = UUID::make_forced("foobarbaz");
    NodePtr node = factory.makeNode("WrappedFunctionNode", node_id);

    ASSERT_TRUE(node != NULL);

    std::vector<Input*> inputs = node->getMessageInputs();
    ASSERT_EQ(2, inputs.size());

    std::vector<Output*> outputs = node->getMessageOutputs();
    ASSERT_EQ(1, outputs.size());

    std::vector<param::Parameter::Ptr> params = node->getParameters();
    ASSERT_EQ(1, params.size());
}

TEST_F(NodeCreationTest, GenericNodeCallsFunctionCorrectly) {
    UUID node_id = UUID::make_forced("foobarbaz2");
    NodePtr node = factory.makeNode("WrappedFunctionNode", node_id);

    ASSERT_TRUE(node != NULL);

    std::vector<Input*> inputs = node->getMessageInputs();
    std::vector<Output*> outputs = node->getMessageOutputs();
    std::vector<param::Parameter::Ptr> params = node->getParameters();

    MockupMessage::Ptr msg(new MockupMessage);
    msg->setSequenceNumber(0);
    msg->value.value = 23;

    MockupMessage::Ptr msg2(new MockupMessage);
    msg2->setSequenceNumber(0);
    msg2->value.value = 17;

    param::Parameter::Ptr param = params[0];
    param->set(2);

    Input* in = inputs[0];
    in->inputMessage(msg);

    Input* in2 = inputs[1];
    in2->inputMessage(msg2);

    // TODO: make this possible:
    /// currently there is a worker spawned automatically!
    //node->process();

    Output* out = outputs[0];
    ConnectionType::Ptr out_msg = out->getMessage();

    ASSERT_TRUE(out_msg != NULL);

    MockupMessage::Ptr result = boost::dynamic_pointer_cast<MockupMessage>(out_msg);

    ASSERT_TRUE(result != NULL);

    ASSERT_EQ(42, result->value.value);
}

}
