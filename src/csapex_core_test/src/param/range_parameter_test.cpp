#include <csapex/param/parameter.h>
#include <csapex/param/value_parameter.h>
#include <csapex/param/range_parameter.h>
#include <csapex/param/string_list_parameter.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/delegate.h>
#include <csapex/serialization/parameter_serializer.h>

#include <unordered_map>
#include <typeindex>
#include <yaml-cpp/yaml.h>

#include <csapex_testing/csapex_test_case.h>

using namespace csapex;
using namespace csapex::param;

class RangeParameterTest : public CsApexTestCase
{
protected:
    RangeParameterTest()
    {
    }
};

TEST_F(RangeParameterTest, HasState)
{
    EXPECT_TRUE(factory::declareRange("a", 10, 100, 10, 1).build()->hasState());
}

TEST_F(RangeParameterTest, StringConversion)
{
    EXPECT_STREQ("[foo: [ranged: 10]]", factory::declareRange("foo", 10, 100, 10, 1).build()->toString().c_str());
    EXPECT_STREQ("[foo: [ranged: 10.5]]", factory::declareRange("foo", 10.5, 100.5, 10.5, 1.0).build()->toString().c_str());
}
template <typename T>
void testSerialization(T min, T max)
{
    YAML::Node node;

    T def = min;

    {
        ParameterPtr p = factory::declareRange<T>("foo", min, max, def, 1);
        p->serialize_yaml(node);
        EXPECT_STREQ(p->name().c_str(), node["name"].as<std::string>().c_str());
        EXPECT_STREQ("range", node["type"].as<std::string>().c_str());
    }

    {
        ParameterPtr p = factory::makeEmpty(node["type"].as<std::string>());
        ASSERT_NE(nullptr, p);
        p->deserialize_yaml(node);

        RangeParameterPtr range = std::dynamic_pointer_cast<RangeParameter>(p);
        ASSERT_NE(nullptr, range);

        EXPECT_STREQ("foo", range->name().c_str());
        EXPECT_EQ(min, range->min<T>());
        EXPECT_EQ(max, range->max<T>());

        auto val = range->as<T>();
        EXPECT_EQ(def, val);
    }

    SerializationBuffer buffer;

    {
        ParameterPtr p = factory::declareRange<T>("foo", min, max, def, 1);
        ParameterSerializer::instance().serialize(*p, buffer);
    }

    {
        StreamablePtr s = ParameterSerializer::instance().deserialize(buffer);
        ASSERT_NE(nullptr, s);

        ParameterPtr p = std::dynamic_pointer_cast<Parameter>(s);
        ASSERT_NE(nullptr, p);

        RangeParameterPtr range = std::dynamic_pointer_cast<RangeParameter>(p);
        ASSERT_NE(nullptr, range);

        EXPECT_STREQ("foo", range->name().c_str());
        EXPECT_EQ(min, range->min<T>());
        EXPECT_EQ(max, range->max<T>());

        auto val = range->as<T>();
        EXPECT_EQ(def, val);
    }
}

TEST_F(RangeParameterTest, Serialization)
{
    testSerialization<int>(10, 100);
    testSerialization<double>(10.5, 100.5);
}

template <typename T>
void testSet(T min, T max)
{
    {
        RangeParameterPtr a = std::make_shared<RangeParameter>();
        ASSERT_ANY_THROW((a->as<std::pair<T, T>>()));
    }

    {
        RangeParameterPtr a = std::make_shared<RangeParameter>();
        a->set(min + 1);

        ASSERT_EQ(min + 1, a->as<T>());

        a->set(min + 2);

        ASSERT_EQ(min + 2, a->as<T>());
    }
}

TEST_F(RangeParameterTest, Set)
{
    testSet<int>(10, 100);
    testSet<double>(10.5, 100.5);
}

template <typename T>
void testAssignment(T min, T max)
{
    RangeParameterPtr a = factory::declareRange<T>("a", min, max, min, 1).template build<RangeParameter>();
    RangeParameterPtr b = factory::declareRange<T>("b", min, max, min, 1).template build<RangeParameter>();

    ASSERT_EQ(min, a->min<T>());
    ASSERT_EQ(max, a->max<T>());
    ASSERT_EQ(min, a->def<T>());

    *b = *a;

    ASSERT_EQ(min, b->min<T>());
    ASSERT_EQ(max, b->max<T>());
    ASSERT_EQ(min, b->def<T>());
}

TEST_F(RangeParameterTest, rangeAssigment)
{
    testAssignment<int>(10, 100);
    testAssignment<double>(10.5, 100.5);
}

template <typename T>
void testDataCloning(T min, T max)
{
    RangeParameterPtr a = factory::declareRange<T>("a", min, max, min, 1).template build<RangeParameter>();
    RangeParameterPtr b = factory::declareRange<T>("b", min, max, min, 1).template build<RangeParameter>();

    ASSERT_EQ(min, a->min<T>());
    ASSERT_EQ(max, a->max<T>());
    ASSERT_EQ(min, a->def<T>());

    b->cloneDataFrom(*a);

    ASSERT_EQ(min, b->min<T>());
    ASSERT_EQ(max, b->max<T>());
    ASSERT_EQ(min, b->def<T>());
}

TEST_F(RangeParameterTest, rangeDataCloning)
{
    testDataCloning<int>(10, 100);
    testDataCloning<double>(10.5, 100.5);
}

template <typename T>
void testCloning(T min, T max)
{
    RangeParameterPtr a = factory::declareRange<T>("a", min, max, min, 1).template build<RangeParameter>();

    ASSERT_EQ(min, a->min<T>());
    ASSERT_EQ(max, a->max<T>());
    ASSERT_EQ(min, a->def<T>());

    RangeParameterPtr b = a->cloneAs<RangeParameter>();

    ASSERT_EQ(min, b->min<T>());
    ASSERT_EQ(max, b->max<T>());
    ASSERT_EQ(min, b->def<T>());

    RangeParameterPtr c = ParameterFactory::clone(a).build<RangeParameter>();

    ASSERT_EQ(min, c->min<T>());
    ASSERT_EQ(max, c->max<T>());
    ASSERT_EQ(min, c->def<T>());
}

TEST_F(RangeParameterTest, rangeCloning)
{
    testCloning<int>(10, 100);
    testCloning<double>(10.5, 100.5);
}
