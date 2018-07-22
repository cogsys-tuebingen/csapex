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
}

TEST_F(IntervalParameterTest, SerializationStringList)
{
    YAML::Node node;

    {
        ParameterPtr p = factory::declareStringList("foo", {"a", "b", "c"});
        p->serialize_yaml(node);
        EXPECT_STREQ(p->name().c_str(), node["name"].as<std::string>().c_str());
        EXPECT_STREQ(p->TYPE().c_str(), node["type"].as<std::string>().c_str());
    }

    {
        ParameterPtr p = factory::makeEmpty(node["type"].as<std::string>());
        ASSERT_NE(nullptr, p);
        p->deserialize_yaml(node);

        ASSERT_FALSE(p->as<std::vector<std::string>>().empty());

        StringListParameterConstPtr sp = std::dynamic_pointer_cast<StringListParameter>(p);
        ASSERT_NE(nullptr, sp);

        EXPECT_EQ(3, sp->count());
        EXPECT_STREQ("foo", p->name().c_str());
        EXPECT_STREQ("a", sp->getValues()[0].c_str());
        EXPECT_STREQ("b", sp->getValues()[1].c_str());
        EXPECT_STREQ("c", sp->getValues()[2].c_str());
    }

    SerializationBuffer buffer;

    {
        ParameterPtr p = factory::declareStringList("foo", {"a", "b", "c"});
        ParameterSerializer::instance().serialize(*p, buffer);
    }

    {
        StreamablePtr s = ParameterSerializer::instance().deserialize(buffer);
        ASSERT_NE(nullptr, s);

        ParameterPtr p = std::dynamic_pointer_cast<Parameter>(s);
        ASSERT_NE(nullptr, p);

        ASSERT_FALSE(p->as<std::vector<std::string>>().empty());

        StringListParameterConstPtr sp = std::dynamic_pointer_cast<StringListParameter>(p);
        ASSERT_NE(nullptr, sp);

        EXPECT_EQ(3, sp->count());
        EXPECT_STREQ("foo", p->name().c_str());
        EXPECT_STREQ("a", sp->getValues()[0].c_str());
        EXPECT_STREQ("b", sp->getValues()[1].c_str());
        EXPECT_STREQ("c", sp->getValues()[2].c_str());
    }
}


TEST_F(IntervalParameterTest, IntervalAssigment)
{
    IntervalParameterPtr a = factory::declareInterval("a", 10, 100, 10, 100, 1).build<IntervalParameter>();
    IntervalParameterPtr b = factory::declareInterval("b", 0, 0, 0, 0, 0).build<IntervalParameter>();

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

TEST_F(IntervalParameterTest, IntervalDataCloning)
{
    IntervalParameterPtr a = factory::declareInterval("a", 10, 100, 10, 100, 1).build<IntervalParameter>();
    IntervalParameterPtr b = factory::declareInterval("b", 0, 0, 0, 0, 0).build<IntervalParameter>();

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

TEST_F(IntervalParameterTest, IntervalCloning)
{
    IntervalParameterPtr a = factory::declareInterval("a", 10, 100, 10, 100, 1).build<IntervalParameter>();

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
