#include <csapex/param/parameter.h>
#include <csapex/param/value_parameter.h>
#include <csapex/param/string_list_parameter.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/delegate.h>
#include <csapex/serialization/parameter_serializer.h>

#include <unordered_map>
#include <typeindex>
#include <yaml-cpp/yaml.h>

#include <csapex_testing/csapex_test_case.h>

using namespace csapex;
using namespace csapex::param;

class ParameterTest : public CsApexTestCase
{
protected:
    ParameterTest()
    {
    }
};

TEST_F(ParameterTest, DescriptionTest)
{
    ParameterPtr a = factory::declareValue("foo", param::ParameterDescription("abc"), 42);
    EXPECT_FALSE(a->description().empty());
    EXPECT_STREQ("abc", a->description().toString().c_str());


    ParameterPtr empty = factory::declareValue("foo", 42);
    EXPECT_TRUE(empty->description().empty());
    EXPECT_STREQ("no description", empty->description().toString().c_str());
}
TEST_F(ParameterTest, TestSpecificSetFrom)
{
    ValueParameterPtr a = factory::declareValue("foo", 42).build<ValueParameter>();
    ValueParameterPtr b = factory::declareValue("bar", 23).build<ValueParameter>();
    ValueParameterPtr c = factory::declareValue("baz", 5).build<ValueParameter>();

    ASSERT_EQ(42, a->as<int>());
    ASSERT_EQ(23, b->as<int>());
    ASSERT_EQ(5, c->as<int>());

    *a = *c;
    ASSERT_EQ(a->as<int>(), c->as<int>());
}
TEST_F(ParameterTest, TestGenericSetFrom)
{
    ParameterPtr a = factory::declareValue("foo", 42);
    ParameterPtr b = factory::declareValue("bar", 23);
    ParameterPtr c = factory::declareValue("baz", 5);

    ASSERT_EQ(42, a->as<int>());
    ASSERT_EQ(23, b->as<int>());
    ASSERT_EQ(5, c->as<int>());

    a->cloneDataFrom(*c);
    ASSERT_EQ(a->as<int>(), c->as<int>());
}
TEST_F(ParameterTest, SetValueDirectly)
{
    ParameterPtr a = factory::declareValue("foo", 42);
    ASSERT_EQ(42, a->as<int>());

    Parameter& a_ref = *a;
    a_ref = 23;
    ASSERT_EQ(23, a_ref.as<int>());
}
TEST_F(ParameterTest, SetStringDirectly)
{
    ParameterPtr a = factory::declareValue("foo", std::string("42"));
    ASSERT_STREQ("42", a->as<std::string>().c_str());

    Parameter& a_ref = *a;
    a_ref = "23";
    ASSERT_STREQ("23", a_ref.as<std::string>().c_str());
}

TEST_F(ParameterTest, InitStringWithCString)
{
    ParameterPtr a = factory::declareValue("foo", "42");
    ASSERT_STREQ("42", a->as<std::string>().c_str());

    Parameter& a_ref = *a;
    a_ref = "23";
    ASSERT_STREQ("23", a_ref.as<std::string>().c_str());
}

TEST_F(ParameterTest, CloningWorksForInt)
{
    ParameterPtr a = factory::declareValue("foo", 42).description("test");

    ASSERT_EQ(42, a->as<int>());
    ASSERT_STREQ("test", a->description().toString().c_str());

    ParameterPtr a_clone = a->cloneAs<Parameter>();
    ASSERT_EQ(42, a_clone->as<int>());
    ASSERT_STREQ("test", a_clone->description().toString().c_str());
}
TEST_F(ParameterTest, CloningWorksWithParameterFactory)
{
    ParameterPtr a = factory::declareValue("foo", 42).description("test");

    ASSERT_EQ(42, a->as<int>());
    ASSERT_STREQ("test", a->description().toString().c_str());

    ParameterPtr a_clone = factory::clone(*a);
    ASSERT_EQ(42, a_clone->as<int>());
    ASSERT_STREQ("test", a_clone->description().toString().c_str());
}

TEST_F(ParameterTest, AssignmentIsPolymorphic)
{
    ValueParameterPtr a = factory::declareValue("a", 42).description("foo").build<ValueParameter>();
    ValueParameterPtr b = factory::declareValue("b", 23).description("bar").build<ValueParameter>();

    ASSERT_EQ(42, a->as<int>());
    ASSERT_EQ(23, b->as<int>());
    ASSERT_STREQ("foo", a->description().toString().c_str());
    ASSERT_STREQ("bar", b->description().toString().c_str());

    *b = *a;

    ASSERT_EQ(42, a->as<int>());
    ASSERT_EQ(42, b->as<int>());
    ASSERT_STREQ("foo", a->description().toString().c_str());
    ASSERT_STREQ("foo", b->description().toString().c_str());
}
