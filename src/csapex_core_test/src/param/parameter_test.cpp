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
TEST_F(ParameterTest, SetValueDirectory)
{
    ParameterPtr a = factory::declareValue("foo", 42);
    ASSERT_EQ(42, a->as<int>());

    Parameter& a_ref = *a;
    a_ref = 23;
    ASSERT_EQ(23, a_ref.as<int>());
}
TEST_F(ParameterTest, SetStringDirectory)
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

TEST_F(ParameterTest, IntervalAssigment)
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

TEST_F(ParameterTest, IntervalDataCloning)
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

TEST_F(ParameterTest, IntervalCloning)
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

TEST_F(ParameterTest, CloningWorksForSet)
{
    enum ColorSpace
    {
        YUV,
        RGB,
        BGR,
        HSL,
        HSV,
        MONO,
        LAB,
        LCh,
        LUV
    };

    std::map<std::string, int> encodings = { { "YUV", (int)YUV }, { "RGB", (int)RGB },   { "BGR", (int)BGR }, { "HSL", (int)HSL },
                                             { "HSV", (int)HSV }, { "MONO", (int)MONO }, { "LAB", (int)LAB }, { "LUV", (int)LUV } };

    ParameterPtr a = csapex::param::factory::declareParameterSet("input", encodings, (int)MONO);
    ASSERT_EQ((int)MONO, a->as<int>());

    ParameterPtr a_clone = a->cloneAs<Parameter>();
    ASSERT_EQ((int)MONO, a_clone->as<int>());
}

TEST_F(ParameterTest, SetValueFromAnotherType)
{
    ParameterPtr value = factory::declareValue("foo", 42.0);

    ASSERT_NEAR(42.0, value->as<double>(), 1e-5);

    ParameterPtr angle = factory::declareAngle("foo_angle", 0.0);
    ASSERT_NEAR(0.0, angle->as<double>(), 1e-5);
    angle->cloneDataFrom(*value);

    ASSERT_NEAR(42.0, angle->as<double>(), 1e-5);
}

TEST_F(ParameterTest, ValueChangedIsTriggered)
{
    ParameterPtr value = factory::declareValue("foo", 42.0);

    bool called = false;
    value->parameter_changed.connect([&](param::Parameter* p) { called = (p == value.get()); });

    ASSERT_NEAR(42.0, value->as<double>(), 1e-5);

    value->set(23.0);

    ASSERT_TRUE(called);
}

TEST_F(ParameterTest, NumberConversion)
{
    ParameterPtr value = factory::declareValue("foo", 42.0);

    ASSERT_NEAR(42.0, value->as<double>(), 1e-5);
    ASSERT_NEAR(42.0f, value->as<float>(), 1e-5);

    ASSERT_THROW(value->as<int>(), std::runtime_error);
}

TEST_F(ParameterTest, HasState)
{
    EXPECT_TRUE(factory::declareValue("foo", 42).build()->hasState());

    EXPECT_FALSE(factory::declareTrigger("foo").build()->hasState());
}

TEST_F(ParameterTest, StringConversion)
{
    // value
    EXPECT_STREQ("[foo: [value: 42]]", factory::declareValue("foo", 42).build()->toString().c_str());
    EXPECT_STREQ("[foo: [value: -42]]", factory::declareValue("foo", -42).build()->toString().c_str());

    EXPECT_STREQ("[foo: [value: 420000000000]]", factory::declareValue("foo", static_cast<long>(420000000000)).build()->toString().c_str());

    EXPECT_STREQ("[foo: [value: -42.000]]", factory::declareValue("foo", -42.0).build()->toString().c_str());
    EXPECT_STREQ("[foo: [value: 42.123]]", factory::declareValue("foo", 42.123).build()->toString().c_str());
    EXPECT_STREQ("[foo: [value: 42.124]]", factory::declareValue("foo", 42.1236).build()->toString().c_str());

    EXPECT_STREQ("[foo: [value: true]]", factory::declareValue("foo", true).build()->toString().c_str());
    EXPECT_STREQ("[foo: [value: false]]", factory::declareValue("foo", false).build()->toString().c_str());

    EXPECT_STREQ("[foo: [value: foobarbaz]]", factory::declareValue("foo", "foobarbaz").build()->toString().c_str());
    EXPECT_STREQ("[foo: [value: ]]", factory::declareValue("foo", "").build()->toString().c_str());


    // trigger
    EXPECT_STREQ("[foo: [trigger]]", factory::declareTrigger("foo").build()->toString().c_str());

    // string list
    EXPECT_STREQ("[foo: [string_list: ]]", factory::declareStringList("foo", {}).build()->toString().c_str());
    EXPECT_STREQ("[foo: [string_list: a]]", factory::declareStringList("foo", {"a"}).build()->toString().c_str());
    EXPECT_STREQ("[foo: [string_list: a, b, c]]", factory::declareStringList("foo", {"a", "b", "c"}).build()->toString().c_str());
}

template <typename T>
void testValueSerialization(T value, const std::string& expected_type)
{
    YAML::Node node;

    {
        ParameterPtr p = factory::declareValue("foo", value);
        p->serialize_yaml(node);
        EXPECT_STREQ(p->name().c_str(), node["name"].as<std::string>().c_str());
        EXPECT_STREQ(p->TYPE().c_str(), node["type"].as<std::string>().c_str());
        ASSERT_TRUE(node[expected_type].IsDefined());
        EXPECT_EQ(value, node[expected_type].as<T>());
    }

    {
        ParameterPtr p = factory::makeEmpty(node["type"].as<std::string>());
        ASSERT_NE(nullptr, p);
        p->deserialize_yaml(node);

        EXPECT_STREQ("foo", p->name().c_str());
        EXPECT_EQ(value, p->as<T>());
    }
}

TEST_F(ParameterTest, YamlSerializationValue)
{
    testValueSerialization<long>(420000000000, "long");
    testValueSerialization<int>(42, "int");
    testValueSerialization<double>(42.0, "double");

    testValueSerialization<bool>(true, "bool");
    testValueSerialization<bool>(false, "bool");

    testValueSerialization<std::string>("foobarbaz", "string");
    testValueSerialization<std::string>("", "string");
}

TEST_F(ParameterTest, SerializationTrigger)
{
    YAML::Node node;

    {
        ParameterPtr p = factory::declareTrigger("foo");
        p->serialize_yaml(node);
        EXPECT_STREQ(p->name().c_str(), node["name"].as<std::string>().c_str());
        EXPECT_STREQ(p->TYPE().c_str(), node["type"].as<std::string>().c_str());
    }

    {
        ParameterPtr p = factory::makeEmpty(node["type"].as<std::string>());
        ASSERT_NE(nullptr, p);
        p->deserialize_yaml(node);
    }

    SerializationBuffer buffer;

    {
        ParameterPtr p = factory::declareTrigger("foo");
        ParameterSerializer::instance().serialize(*p, buffer);
    }

    {
        StreamablePtr s = ParameterSerializer::instance().deserialize(buffer);
        ASSERT_NE(nullptr, s);

        ParameterPtr p = std::dynamic_pointer_cast<Parameter>(s);
        ASSERT_NE(nullptr, p);

        TriggerParameterConstPtr sp = std::dynamic_pointer_cast<TriggerParameter>(p);
        ASSERT_NE(nullptr, sp);
    }
}

TEST_F(ParameterTest, SerializationStringList)
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


template <typename T>
void testSetValue(T def, T value, const std::string& expected_type)
{
    ASSERT_NE(def, value);
    ParameterPtr p = factory::declareValue("foo", def);
    EXPECT_EQ(def, p->as<T>());
    p->set(value);
    EXPECT_EQ(value, p->as<T>());

    // cloning transfers the value
    ParameterPtr pclone = factory::clone(p.get());
    EXPECT_EQ(pclone->as<T>(), p->as<T>());

    // check if parameter_changed will be called
    bool called = false;
    pclone->parameter_changed.connect([&called](){
        called = true;
    });

    // clone identical parameter -> parameter_changed should not be called
    pclone->cloneDataFrom(*p);
    EXPECT_EQ(pclone->as<T>(), p->as<T>());
    ASSERT_FALSE(called);

    // clone different parameter -> parameter_changed should not called
    ParameterPtr p2 = factory::declareValue("foo", def);
    pclone->cloneDataFrom(*p2);
    EXPECT_EQ(pclone->as<T>(), p2->as<T>());
    ASSERT_TRUE(called);
}

TEST_F(ParameterTest, SetValue)
{
    testSetValue<long>(0, 420000000000, "long");
    testSetValue<int>(0, 42, "int");
    testSetValue<double>(0, 42.0, "double");

    testSetValue<bool>(false, true, "bool");
    testSetValue<bool>(true, false, "bool");

    testSetValue<std::string>("", "foobarbaz", "string");
    testSetValue<std::string>("...", "", "string");
}

TEST_F(ParameterTest, SetValueFromRange)
{
    ParameterPtr range = factory::declareRange("range", 0, 100, 50, 1);
    ParameterPtr value = factory::declareValue("foo", 0);

    value->cloneDataFrom(*range);

    ASSERT_EQ(50, value->as<int>());
}


TEST_F(ParameterTest, SetValueFromTriggerThrows)
{
    ParameterPtr trigger = factory::declareTrigger("range");
    ParameterPtr value = factory::declareValue("foo", 0);

    ASSERT_ANY_THROW(value->cloneDataFrom(*trigger));
}


TEST_F(ParameterTest, InvalidCloneFromThrows)
{
    ParameterPtr target = std::make_shared<StringListParameter>();
    ParameterPtr source = std::make_shared<ValueParameter>();

    ASSERT_ANY_THROW(target->cloneDataFrom(*source));
}



TEST_F(ParameterTest, SettingTriggerCausesTrigger)
{
    ParameterPtr trigger = factory::declareTrigger("range");
    bool called = false;
    trigger->parameter_changed.connect([&called](){
        called = true;
    });

    trigger->set(42);
    ASSERT_TRUE(called);

    // cloning does not keep the old signals
    called = false;
    ParameterPtr trigger2 = factory::clone(trigger);
    trigger2->set(42);
    ASSERT_FALSE(called);
}

TEST_F(ParameterTest, StringListAccess)
{
    StringListParameterPtr list =  factory::declareStringList("foo", {"a", "b", "c"}).build<StringListParameter>();

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
