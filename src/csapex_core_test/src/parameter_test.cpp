#include <csapex/param/parameter.h>
#include <csapex/param/value_parameter.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/delegate.h>

#include <unordered_map>
#include <typeindex>

#include "gtest/gtest.h"

using namespace csapex;
using namespace csapex::param;

class ParameterTest : public ::testing::Test
{
protected:
    ParameterTest()
    {
    }
};


TEST_F(ParameterTest, TestSpecificSetFrom)
{
    ValueParameterPtr a = ParameterFactory::declareValue("foo", 42).build<ValueParameter>();
    ValueParameterPtr b = ParameterFactory::declareValue("bar", 23).build<ValueParameter>();
    ValueParameterPtr c = ParameterFactory::declareValue("baz", 5).build<ValueParameter>();

    ASSERT_EQ(42, a->as<int>());
    ASSERT_EQ(23, b->as<int>());
    ASSERT_EQ(5, c->as<int>());

    *a = *c;
    ASSERT_EQ(a->as<int>(), c->as<int>());
}
TEST_F(ParameterTest, TestGenericSetFrom)
{
    ParameterPtr a = ParameterFactory::declareValue("foo", 42);
    ParameterPtr b = ParameterFactory::declareValue("bar", 23);
    ParameterPtr c = ParameterFactory::declareValue("baz", 5);

    ASSERT_EQ(42, a->as<int>());
    ASSERT_EQ(23, b->as<int>());
    ASSERT_EQ(5, c->as<int>());

    a->cloneDataFrom(*c);
    ASSERT_EQ(a->as<int>(), c->as<int>());
}
TEST_F(ParameterTest, SetValueDirectory)
{
    ParameterPtr a = ParameterFactory::declareValue("foo", 42);
    ASSERT_EQ(42, a->as<int>());

    Parameter& a_ref = *a;
    a_ref = 23;
    ASSERT_EQ(23, a_ref.as<int>());
}
TEST_F(ParameterTest, SetStringDirectory)
{
    ParameterPtr a = ParameterFactory::declareValue("foo", std::string("42"));
    ASSERT_STREQ("42", a->as<std::string>().c_str());

    Parameter& a_ref = *a;
    a_ref = "23";
    ASSERT_STREQ("23", a_ref.as<std::string>().c_str());
}

TEST_F(ParameterTest, InitStringWithCString)
{
    ParameterPtr a = ParameterFactory::declareValue("foo", "42");
    ASSERT_STREQ("42", a->as<std::string>().c_str());

    Parameter& a_ref = *a;
    a_ref = "23";
    ASSERT_STREQ("23", a_ref.as<std::string>().c_str());
}

TEST_F(ParameterTest, CloningWorksForInt)
{
    ParameterPtr a = ParameterFactory::declareValue("foo", 42).description("test");

    ASSERT_EQ(42, a->as<int>());
    ASSERT_STREQ("test", a->description().toString().c_str());

    ParameterPtr a_clone = a->cloneAs<Parameter>();
    ASSERT_EQ(42, a_clone->as<int>());
    ASSERT_STREQ("test", a_clone->description().toString().c_str());
}
TEST_F(ParameterTest, CloningWorksWithParameterFactory)
{
    ParameterPtr a = ParameterFactory::declareValue("foo", 42).description("test");

    ASSERT_EQ(42, a->as<int>());
    ASSERT_STREQ("test", a->description().toString().c_str());

    ParameterPtr a_clone = ParameterFactory::clone(*a);
    ASSERT_EQ(42, a_clone->as<int>());
    ASSERT_STREQ("test", a_clone->description().toString().c_str());
}

TEST_F(ParameterTest, AssignmentIsPolymorphic)
{
    ValueParameterPtr a = ParameterFactory::declareValue("a", 42).description("foo").build<ValueParameter>();
    ValueParameterPtr b = ParameterFactory::declareValue("b", 23).description("bar").build<ValueParameter>();

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

TEST_F(ParameterTest, IntervalAssigment)
{
    IntervalParameterPtr a = ParameterFactory::declareInterval("a", 10, 100, 10, 100, 1).build<IntervalParameter>();
    IntervalParameterPtr b = ParameterFactory::declareInterval("b", 0, 0, 0, 0, 0).build<IntervalParameter>();

    ASSERT_EQ(10, a->min<int>());
    ASSERT_EQ(100, a->max<int>());
    ASSERT_EQ(10, a->def<int>().first);
    ASSERT_EQ(100, a->def<int>().second);

    *b = *a;

    ASSERT_EQ(10, b->min<int>());
    ASSERT_EQ(100, b->max<int>());
    ASSERT_EQ(10, b->def<int>().first);
    ASSERT_EQ(100, b->def<int>().second);
}


TEST_F(ParameterTest, IntervalDataCloning)
{
    IntervalParameterPtr a = ParameterFactory::declareInterval("a", 10, 100, 10, 100, 1).build<IntervalParameter>();
    IntervalParameterPtr b = ParameterFactory::declareInterval("b", 0, 0, 0, 0, 0).build<IntervalParameter>();

    ASSERT_EQ(10, a->min<int>());
    ASSERT_EQ(100, a->max<int>());
    ASSERT_EQ(10, a->def<int>().first);
    ASSERT_EQ(100, a->def<int>().second);

    b->cloneDataFrom(*a);

    ASSERT_EQ(10, b->min<int>());
    ASSERT_EQ(100, b->max<int>());
    ASSERT_EQ(10, b->def<int>().first);
    ASSERT_EQ(100, b->def<int>().second);
}

TEST_F(ParameterTest, IntervalCloning)
{
    IntervalParameterPtr a = ParameterFactory::declareInterval("a", 10, 100, 10, 100, 1).build<IntervalParameter>();

    ASSERT_EQ(10, a->min<int>());
    ASSERT_EQ(100, a->max<int>());
    ASSERT_EQ(10, a->def<int>().first);
    ASSERT_EQ(100, a->def<int>().second);

    IntervalParameterPtr b = a->cloneAs<IntervalParameter>();

    ASSERT_EQ(10, b->min<int>());
    ASSERT_EQ(100, b->max<int>());
    ASSERT_EQ(10, b->def<int>().first);
    ASSERT_EQ(100, b->def<int>().second);
}

TEST_F(ParameterTest, CloningWorksForSet)
{
    enum ColorSpace {YUV, RGB, BGR, HSL, HSV, MONO, LAB, LCh, LUV};

    std::map<std::string, int> encodings = {
        {"YUV", (int) YUV},
        {"RGB", (int) RGB},
        {"BGR", (int) BGR},
        {"HSL", (int) HSL},
        {"HSV", (int) HSV},
        {"MONO",(int) MONO},
        {"LAB", (int) LAB},
        {"LUV" ,(int) LUV}
    };

    ParameterPtr a = csapex::param::ParameterFactory::declareParameterSet("input", encodings, (int) MONO);
    ASSERT_EQ((int) MONO, a->as<int>());

    ParameterPtr a_clone = a->cloneAs<Parameter>();
    ASSERT_EQ((int) MONO, a_clone->as<int>());
}

TEST_F(ParameterTest, SetValueFromAnotherType)
{
    ParameterPtr value = ParameterFactory::declareValue("foo", 42.0);

    ASSERT_NEAR(42.0, value->as<double>(), 1e-5);

    ParameterPtr angle = ParameterFactory::declareAngle("foo_angle", 0.0);
    ASSERT_NEAR(0.0, angle->as<double>(), 1e-5);
    angle->cloneDataFrom(*value);

    ASSERT_NEAR(42.0, angle->as<double>(), 1e-5);
}

TEST_F(ParameterTest, ValueChangedIsTriggered)
{
    ParameterPtr value = ParameterFactory::declareValue("foo", 42.0);

    bool called = false;
    value->parameter_changed.connect([&](param::Parameter* p) {
       called = (p == value.get());
    });

    ASSERT_NEAR(42.0, value->as<double>(), 1e-5);

    value->set(23.0);

    ASSERT_TRUE(called);
}

TEST_F(ParameterTest, NumberConversion)
{
    ParameterPtr value = ParameterFactory::declareValue("foo", 42.0);

    ASSERT_NEAR(42.0, value->as<double>(), 1e-5);
    ASSERT_NEAR(42.0f, value->as<float>(), 1e-5);

    ASSERT_THROW(value->as<int>(), std::runtime_error);
}
