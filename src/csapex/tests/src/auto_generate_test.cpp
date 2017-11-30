#include <csapex/model/node.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/factory/generic_node_factory.hpp>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/model/node_constructor.h>
#include <csapex/msg/marker_message.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/model/token.h>
#include <csapex/core/settings/settings_impl.h>

#include "gtest/gtest.h"

#include <type_traits>

using namespace csapex;
using namespace connection_types;

namespace csapex {


void f1(const GenericValueMessage<int>& input, const GenericValueMessage<int>& input2, int parameter, GenericValueMessage<int>& output)
{
    output.value = input.value + input2.value + parameter;
}
void f2(const int& input, const int& input2, int parameter, int& output)
{
    output = input + input2 + parameter;
}
void f3(int parameter1,
        std::string& output,
        const int& input, const double& input2, int parameter2, int parameter3 )
{
    output = std::to_string(static_cast<int>(input + input2 + parameter1 + parameter2 + parameter3));
}
}


class AutoGenerateTest : public ::testing::Test {
protected:
    NodeFactoryImplementation factory;

    AutoGenerateTest()
        : factory(SettingsImplementation::NoSettings, nullptr), uuid_provider(std::make_shared<UUIDProvider>())
    {
    }

    virtual ~AutoGenerateTest() {
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

    UUIDProviderPtr uuid_provider;
};

TEST_F(AutoGenerateTest, ExplicitTypesAreDetected) {
    factory.registerNodeType(GenericNodeFactory::createConstructorFromFunction(f1, "f1"));

    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");
    NodeFacadeImplementationPtr node = factory.makeNode("f1", node_id, uuid_provider);

    ASSERT_TRUE(node != nullptr);
    EXPECT_EQ(node_id, node->getUUID());

    EXPECT_EQ(node->getParameters().size(), 1);

    EXPECT_EQ(node->getInputs().size(), 2 + node->getParameters().size());
    EXPECT_EQ(node->getOutputs().size(), 1 + node->getParameters().size());

    GenericValueMessage<int>::Ptr a(new GenericValueMessage<int>);
    GenericValueMessage<int>::Ptr b(new GenericValueMessage<int>);

    a->value = 23;
    b->value = 42;

    InputPtr i1 = node->getNodeHandle()->getInput(UUIDProvider::makeDerivedUUID_forced(node_id, "in_0"));
    ASSERT_TRUE(i1 != nullptr);

    InputPtr i2 = node->getNodeHandle()->getInput(UUIDProvider::makeDerivedUUID_forced(node_id, "in_1"));
    ASSERT_TRUE(i2 != nullptr);

    TokenPtr ta = std::make_shared<Token>(a);
    TokenPtr tb = std::make_shared<Token>(b);

    param::ParameterPtr p = node->getParameter("param 2");
    ASSERT_TRUE(p != nullptr);

    p->set<int>(1337);

    param::ValueParameter::Ptr vp = std::dynamic_pointer_cast<param::ValueParameter>(p);
    ASSERT_TRUE(vp != nullptr);

    i1->setToken(ta);
    i2->setToken(tb);

    NodePtr n = node->getNode();
    n->process(*node->getNodeHandle(), *n);

    OutputPtr o = node->getNodeHandle()->getOutput(UUIDProvider::makeDerivedUUID_forced(node_id, "out_0"));
    ASSERT_TRUE(o != nullptr);

    o->commitMessages(false);

    TokenPtr to = o->getToken();
    ASSERT_TRUE(to != nullptr);
    ASSERT_TRUE(to->getTokenData() != nullptr);

    GenericValueMessage<int>::ConstPtr result = std::dynamic_pointer_cast<GenericValueMessage<int> const>(to->getTokenData());
    ASSERT_TRUE(result != nullptr);

    EXPECT_EQ((a->value + b->value + vp->as<int>()), result->value);
}


TEST_F(AutoGenerateTest, ImplicitTypesAreDetected) {
    factory.registerNodeType(GenericNodeFactory::createConstructorFromFunction(f2, "f2"));

    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");
    NodeFacadeImplementationPtr node = factory.makeNode("f2", node_id, uuid_provider);

    ASSERT_TRUE(node != nullptr);
    EXPECT_EQ(node_id, node->getUUID());

    EXPECT_EQ(node->getParameters().size(), 1);

    EXPECT_EQ(node->getInputs().size(), 2 + node->getParameters().size());
    EXPECT_EQ(node->getOutputs().size(), 1 + node->getParameters().size());

    GenericValueMessage<int>::Ptr a(new GenericValueMessage<int>);
    GenericValueMessage<int>::Ptr b(new GenericValueMessage<int>);

    a->value = 23;
    b->value = 42;

    InputPtr i1 = node->getNodeHandle()->getInput(UUIDProvider::makeDerivedUUID_forced(node_id, "in_0"));
    ASSERT_TRUE(i1 != nullptr);

    InputPtr i2 = node->getNodeHandle()->getInput(UUIDProvider::makeDerivedUUID_forced(node_id, "in_1"));
    ASSERT_TRUE(i2 != nullptr);

    TokenPtr ta = std::make_shared<Token>(a);
    TokenPtr tb = std::make_shared<Token>(b);

    param::ParameterPtr p = node->getParameter("param 2");
    ASSERT_TRUE(p != nullptr);

    p->set<int>(1337);

    param::ValueParameter::Ptr vp = std::dynamic_pointer_cast<param::ValueParameter>(p);
    ASSERT_TRUE(vp != nullptr);

    i1->setToken(ta);
    i2->setToken(tb);

    NodePtr n = node->getNode();
    n->process(*node->getNodeHandle(), *n);

    OutputPtr o = node->getNodeHandle()->getOutput(UUIDProvider::makeDerivedUUID_forced(node_id, "out_0"));
    ASSERT_TRUE(o != nullptr);

    o->commitMessages(false);

    TokenPtr to = o->getToken();
    ASSERT_TRUE(to != nullptr);
    ASSERT_TRUE(to->getTokenData() != nullptr);

    GenericValueMessage<int>::ConstPtr result = std::dynamic_pointer_cast<GenericValueMessage<int> const>(to->getTokenData());
    ASSERT_TRUE(result != nullptr);

    EXPECT_EQ((a->value + b->value + vp->as<int>()), result->value);
}


TEST_F(AutoGenerateTest, OrderDoesNotMatterTypesAreDetected) {
    factory.registerNodeType(GenericNodeFactory::createConstructorFromFunction(f3, "f3"));

    UUID node_id = UUIDProvider::makeUUID_without_parent("foobarbaz");
    NodeFacadeImplementationPtr node = factory.makeNode("f3", node_id, uuid_provider);

    ASSERT_TRUE(node != nullptr);
    EXPECT_EQ(node_id, node->getUUID());

    EXPECT_EQ(node->getParameters().size(), 3);

    EXPECT_EQ(node->getInputs().size(), 2 + node->getParameters().size());
    EXPECT_EQ(node->getOutputs().size(), 1 + node->getParameters().size());

    GenericValueMessage<int>::Ptr a(new GenericValueMessage<int>);
    GenericValueMessage<double>::Ptr b(new GenericValueMessage<double>);

    a->value = 23;
    b->value = 42.0;

    InputPtr i1 = node->getNodeHandle()->getInput(UUIDProvider::makeDerivedUUID_forced(node_id, "in_0"));
    ASSERT_TRUE(i1 != nullptr);

    InputPtr i2 = node->getNodeHandle()->getInput(UUIDProvider::makeDerivedUUID_forced(node_id, "in_1"));
    ASSERT_TRUE(i2 != nullptr);

    TokenPtr ta = std::make_shared<Token>(a);
    TokenPtr tb = std::make_shared<Token>(b);

    param::ParameterPtr p_1 = node->getParameter("param 0");
    ASSERT_TRUE(p_1 != nullptr);
    p_1->set<int>(1337);

    param::ParameterPtr p_2 = node->getParameter("param 4");
    ASSERT_TRUE(p_2 != nullptr);
    p_2->set<int>(4);

    param::ParameterPtr p_3 = node->getParameter("param 5");
    ASSERT_TRUE(p_3 != nullptr);
    p_3->set<int>(5);

    i1->setToken(ta);
    i2->setToken(tb);

    NodePtr n = node->getNode();
    n->process(*node->getNodeHandle(), *n);

    OutputPtr o = node->getNodeHandle()->getOutput(UUIDProvider::makeDerivedUUID_forced(node_id, "out_0"));
    ASSERT_TRUE(o != nullptr);

    o->commitMessages(false);

    TokenPtr to = o->getToken();
    ASSERT_TRUE(to != nullptr);
    ASSERT_TRUE(to->getTokenData() != nullptr);

    GenericValueMessage<std::string>::ConstPtr result = std::dynamic_pointer_cast<GenericValueMessage<std::string> const>(to->getTokenData());
    ASSERT_TRUE(result != nullptr);

    EXPECT_EQ("1411", result->value);
}
