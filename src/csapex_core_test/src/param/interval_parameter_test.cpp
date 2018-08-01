#include <csapex/param/parameter.h>
#include <csapex/param/value_parameter.h>
#include <csapex/param/interval_parameter.h>
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

class IntervalParameterTest : public CsApexTestCase
{
protected:
    IntervalParameterTest()
    {
    }
};

TEST_F(IntervalParameterTest, HasState)
{
    EXPECT_TRUE(factory::declareInterval("a", 10, 100, 10, 100, 1).build()->hasState());
}

TEST_F(IntervalParameterTest, StringConversion)
{
    EXPECT_STREQ("[foo: [interval: 10 : 100]]", factory::declareInterval("foo", 10, 100, 10, 100, 1).build()->toString().c_str());
    EXPECT_STREQ("[foo: [interval: 10.5 : 100.5]]", factory::declareInterval("foo", 10.5, 100.5, 10.5, 100.5, 1.0).build()->toString().c_str());
}
template <typename T>
void testSerialization(T min, T max)
{

    YAML::Node node;

    {
        ParameterPtr p = factory::declareInterval<T>("foo", min, max, min, max, 1);
        p->serialize_yaml(node);
        EXPECT_STREQ(p->name().c_str(), node["name"].as<std::string>().c_str());
        EXPECT_STREQ(p->TYPE().c_str(), node["type"].as<std::string>().c_str());
    }

    {
        ParameterPtr p = factory::makeEmpty(node["type"].as<std::string>());
        ASSERT_NE(nullptr, p);
        p->deserialize_yaml(node);

        IntervalParameterPtr interval = std::dynamic_pointer_cast<IntervalParameter>(p);
        ASSERT_NE(nullptr, interval);

        EXPECT_STREQ("foo", interval->name().c_str());
        EXPECT_EQ(min, interval->min<T>());
        EXPECT_EQ(max, interval->max<T>());

        auto val = interval->as<std::pair<T,T>>();
        EXPECT_EQ(min, val.first);
        EXPECT_EQ(max, val.second);
    }

    SerializationBuffer buffer;

    {
        ParameterPtr p = factory::declareInterval<T>("foo", min, max, min, max, 1);
        ParameterSerializer::instance().serialize(*p, buffer);
    }

    {
        StreamablePtr s = ParameterSerializer::instance().deserialize(buffer);
        ASSERT_NE(nullptr, s);

        ParameterPtr p = std::dynamic_pointer_cast<Parameter>(s);
        ASSERT_NE(nullptr, p);

        IntervalParameterPtr interval = std::dynamic_pointer_cast<IntervalParameter>(p);
        ASSERT_NE(nullptr, interval);

        EXPECT_STREQ("foo", interval->name().c_str());
        EXPECT_EQ(min, interval->min<T>());
        EXPECT_EQ(max, interval->max<T>());
    }
}

TEST_F(IntervalParameterTest, Serialization)
{
    testSerialization<int>(10, 100);
    testSerialization<double>(10.5, 100.5);
}

template <typename T>
void testSet(T min, T max)
{
    {
        IntervalParameterPtr a = std::make_shared<IntervalParameter>();
        ASSERT_ANY_THROW((a->as<std::pair<T,T>>()));
    }

    {
        IntervalParameterPtr a = std::make_shared<IntervalParameter>();
        a->set(std::make_pair(min+1, max-1));

        ASSERT_EQ(min+1, a->lower<T>());
        ASSERT_EQ(max-1, a->upper<T>());

        a->set(std::make_pair(min+2, max-2));

        ASSERT_EQ(min+2, a->lower<T>());
        ASSERT_EQ(max-2, a->upper<T>());
    }
}

TEST_F(IntervalParameterTest, Set)
{
    testSet<int>(10, 100);
    testSet<double>(10.5, 100.5);
}

template <typename T>
void testAssignment(T min, T max)
{
    IntervalParameterPtr a = factory::declareInterval<T>("a", min, max, min, max, 1).template build<IntervalParameter>();
    IntervalParameterPtr b = factory::declareInterval<T>("b", min, max, min, max, 0).template build<IntervalParameter>();

    ASSERT_EQ(min, a->min<T>());
    ASSERT_EQ(max, a->max<T>());
    ASSERT_EQ(min, a->def<T>().first);
    ASSERT_EQ(max, a->def<T>().second);

    *b = *a;

    ASSERT_EQ(min, b->min<T>());
    ASSERT_EQ(max, b->max<T>());
    ASSERT_EQ(min, b->def<T>().first);
    ASSERT_EQ(max, b->def<T>().second);
}

TEST_F(IntervalParameterTest, IntervalAssigment)
{
    testAssignment<int>(10, 100);
    testAssignment<double>(10.5, 100.5);
}

template <typename T>
void testDataCloning(T min, T max)
{
    IntervalParameterPtr a = factory::declareInterval<T>("a", min, max, min, max, 1).template build<IntervalParameter>();
    IntervalParameterPtr b = factory::declareInterval<T>("b", min, max, min, max, 0).template build<IntervalParameter>();

    ASSERT_EQ(min, a->min<T>());
    ASSERT_EQ(max, a->max<T>());
    ASSERT_EQ(min, a->def<T>().first);
    ASSERT_EQ(max, a->def<T>().second);

    b->cloneDataFrom(*a);

    ASSERT_EQ(min, b->min<T>());
    ASSERT_EQ(max, b->max<T>());
    ASSERT_EQ(min, b->def<T>().first);
    ASSERT_EQ(max, b->def<T>().second);
}

TEST_F(IntervalParameterTest, IntervalDataCloning)
{
    testDataCloning<int>(10, 100);
    testDataCloning<double>(10.5, 100.5);
}

template <typename T>
void testCloning(T min, T max)
{
    IntervalParameterPtr a = factory::declareInterval<T>("a", min, max, min, max, 1).template build<IntervalParameter>();

    ASSERT_EQ(min, a->min<T>());
    ASSERT_EQ(max, a->max<T>());
    ASSERT_EQ(min, a->def<T>().first);
    ASSERT_EQ(max, a->def<T>().second);

    IntervalParameterPtr b = a->cloneAs<IntervalParameter>();

    ASSERT_EQ(min, b->min<T>());
    ASSERT_EQ(max, b->max<T>());
    ASSERT_EQ(min, b->def<T>().first);
    ASSERT_EQ(max, b->def<T>().second);
}


TEST_F(IntervalParameterTest, IntervalCloning)
{
    testCloning<int>(10, 100);
    testCloning<double>(10.5, 100.5);
}
