#include <csapex/param/parameter.h>
#include <csapex/param/value_parameter.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/param/string_list_parameter.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/delegate.h>
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/utility/yaml.h>

#include <csapex_testing/csapex_test_case.h>

#include <unordered_map>
#include <typeindex>

using namespace csapex;
using namespace csapex::param;

class StringListParameterTest : public CsApexTestCase
{
protected:
    StringListParameterTest()
    {
    }
};

TEST_F(StringListParameterTest, HasState)
{
    EXPECT_TRUE(factory::declareStringList("foo", {}).build()->hasState());
}

TEST_F(StringListParameterTest, StringConversion)
{
    EXPECT_STREQ("[foo: [string_list: ]]", factory::declareStringList("foo", {}).build()->toString().c_str());
    EXPECT_STREQ("[foo: [string_list: a]]", factory::declareStringList("foo", { "a" }).build()->toString().c_str());
    EXPECT_STREQ("[foo: [string_list: a, b, c]]", factory::declareStringList("foo", { "a", "b", "c" }).build()->toString().c_str());
}

TEST_F(StringListParameterTest, SerializationStringList)
{
    YAML::Node node;

    {
        ParameterPtr p = factory::declareStringList("foo", { "a", "b", "c" });
        p->serialize_yaml(node);
        EXPECT_STREQ(p->name().c_str(), node["name"].as<std::string>().c_str());
        EXPECT_STREQ(p->getParameterType().c_str(), node["type"].as<std::string>().c_str());
    }

    {
        ParameterPtr p = factory::makeEmpty(node["type"].as<std::string>());
        ASSERT_NE(nullptr, p);
        p->deserialize_yaml(node);

        ASSERT_FALSE(p->as<std::vector<std::string>>().empty());

        StringListParameterConstPtr sp = std::dynamic_pointer_cast<StringListParameter>(p);
        ASSERT_NE(nullptr, sp);

        EXPECT_EQ(3u, sp->count());
        EXPECT_STREQ("foo", p->name().c_str());
        EXPECT_STREQ("a", sp->getValues()[0].c_str());
        EXPECT_STREQ("b", sp->getValues()[1].c_str());
        EXPECT_STREQ("c", sp->getValues()[2].c_str());
    }

    SerializationBuffer buffer;

    {
        ParameterPtr p = factory::declareStringList("foo", { "a", "b", "c" });
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

TEST_F(StringListParameterTest, StringListAccess)
{
    StringListParameterPtr list = factory::declareStringList("foo", { "a", "b", "c" }).build<StringListParameter>();

    EXPECT_EQ(3, list->count());
    EXPECT_STREQ("c", list->getValues()[2].c_str());

    list->setAt(2, "x");
    EXPECT_STREQ("x", list->getValues()[2].c_str());

    EXPECT_STREQ("a", list->getValues()[0].c_str());
    list->remove(0);
    EXPECT_STREQ("b", list->getValues()[0].c_str());

    EXPECT_EQ(2, list->count());
    list->add("R");
    list->add("R");
    list->add("R");
    EXPECT_EQ(5, list->count());
    list->removeAll("R");
    EXPECT_EQ(2, list->count());
}

TEST_F(StringListParameterTest, InvalidCloneFromThrows)
{
    ParameterPtr target = std::make_shared<StringListParameter>();
    ParameterPtr source = std::make_shared<ValueParameter>();

    ASSERT_ANY_THROW(target->cloneDataFrom(*source));
}
