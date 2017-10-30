#include <csapex/model/node.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/factory/node_factory_local.h>
#include <csapex/factory/generic_node_factory.hpp>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/model/node_constructor.h>
#include <csapex/msg/marker_message.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/model/token.h>
#include <csapex/core/settings/settings_local.h>

#include "gtest/gtest.h"

using namespace csapex;
using namespace connection_types;

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

void functionToBeWrappedIntoANode(const GenericValueMessage<int>& input, const GenericValueMessage<int>& input2, int parameter, GenericValueMessage<int>& output)
{
    output.value = input.value + input2.value + parameter;
}
}


class NodeCreationTest : public ::testing::Test {
protected:
    NodeFactoryLocal factory;

    NodeCreationTest()
        : factory(SettingsLocal::NoSettings, nullptr), uuid_provider(std::make_shared<UUIDProvider>())
    {
        csapex::NodeConstructor::Ptr mockup_constructor(new csapex::NodeConstructor("MockupNode", std::bind(&NodeCreationTest::makeMockup)));
        factory.registerNodeType(mockup_constructor);

        factory.registerNodeType(GenericNodeFactory::createConstructorFromFunction(functionToBeWrappedIntoANode, "WrappedFunctionNode"));
    }

    virtual ~NodeCreationTest() {
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

    UUIDProviderPtr uuid_provider;
};

TEST_F(NodeCreationTest, NodeCanBeMadeInAFactory) {
    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");
    NodeFacadeLocalPtr node = factory.makeNode("MockupNode", node_id, uuid_provider);

    ASSERT_TRUE(node != nullptr);
    EXPECT_EQ(node_id, node->getUUID());
}

TEST_F(NodeCreationTest, GenericNodeCanBeMadeInAFactory) {
    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");
    NodeFacadeLocalPtr node_facade = factory.makeNode("WrappedFunctionNode", node_id, uuid_provider);
    ASSERT_TRUE(node_facade != nullptr);

    ASSERT_TRUE(node_facade->getNodeHandle() != nullptr);

    NodePtr node = node_facade->getNode();
    ASSERT_TRUE(node != nullptr);

    std::vector<InputPtr> inputs = node_facade->getNodeHandle()->getExternalInputs();
    ASSERT_EQ(2 + 1, inputs.size());

    std::vector<OutputPtr> outputs = node_facade->getNodeHandle()->getExternalOutputs();
    ASSERT_EQ(1 + 1, outputs.size());

    std::vector<csapex::param::Parameter::Ptr> params = node->getParameters();
    ASSERT_EQ(1, params.size());
}

TEST_F(NodeCreationTest, GenericNodeCallsFunctionCorrectly) {
    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz2");

    NodeFacadeLocalPtr node_facade = factory.makeNode("WrappedFunctionNode", node_id, uuid_provider);
    ASSERT_TRUE(node_facade != nullptr);

    NodePtr node = node_facade->getNode();
    ASSERT_TRUE(node != nullptr);

    std::vector<InputPtr> inputs = node_facade->getNodeHandle()->getExternalInputs();
    ASSERT_EQ(2 + 1, inputs.size());

    std::vector<OutputPtr> outputs = node_facade->getNodeHandle()->getExternalOutputs();
    ASSERT_EQ(1 + 1, outputs.size());

    std::vector<csapex::param::Parameter::Ptr> params = node->getParameters();
    ASSERT_EQ(1, params.size());

    GenericValueMessage<int>::Ptr msg(new GenericValueMessage<int>);
    msg->value = 23;

    GenericValueMessage<int>::Ptr msg2(new GenericValueMessage<int>);
    msg2->value = 17;

    csapex::param::Parameter::Ptr param = params[0];
    param->set(2);

    //InputPtr param_in = inputs[0];

    InputPtr in = inputs[1];
    in->setToken(std::make_shared<Token>(msg));
    ASSERT_TRUE(in->getToken() != nullptr);

    InputPtr in2 = inputs[2];
    in2->setToken(std::make_shared<Token>(msg2));
    ASSERT_TRUE(in2->getToken() != nullptr);

    node->process(*node_facade->getNodeHandle(), *node);

    //OutputPtr param_out = outputs[0];

    OutputPtr out = outputs[1];
    out->commitMessages(false); // this smells a little..

    TokenConstPtr out_msg = out->getToken();
    ASSERT_TRUE(out_msg != nullptr);

    MarkerMessage::ConstPtr nothing = std::dynamic_pointer_cast<MarkerMessage const>(out_msg->getTokenData());
    ASSERT_TRUE(nothing == nullptr);

    GenericValueMessage<int>::ConstPtr result = std::dynamic_pointer_cast<GenericValueMessage<int> const>(out_msg->getTokenData());
    ASSERT_TRUE(result != nullptr);

    ASSERT_EQ(42, result->value);
}
