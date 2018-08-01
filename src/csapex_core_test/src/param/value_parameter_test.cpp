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

class ValueParameterTest : public CsApexTestCase
{
protected:
    ValueParameterTest()
    {
    }
};


TEST_F(ValueParameterTest, HasState)
{
    EXPECT_TRUE(factory::declareValue("foo", 42).build()->hasState());
}

TEST_F(ValueParameterTest, StringConversion)
{
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
}

template <typename T>
void testSerialization(T value, const std::string& expected_type)
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

TEST_F(ValueParameterTest, YamlSerializationValue)
{
    testSerialization<long>(420000000000, "long");
    testSerialization<int>(42, "int");
    testSerialization<double>(42.0, "double");

    testSerialization<bool>(true, "bool");
    testSerialization<bool>(false, "bool");

    testSerialization<std::string>("foobarbaz", "string");
    testSerialization<std::string>("", "string");
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

TEST_F(ValueParameterTest, SetValue)
{
    testSetValue<long>(0, 420000000000, "long");
    testSetValue<int>(0, 42, "int");
    testSetValue<double>(0, 42.0, "double");

    testSetValue<bool>(false, true, "bool");
    testSetValue<bool>(true, false, "bool");

    testSetValue<std::string>("", "foobarbaz", "string");
    testSetValue<std::string>("...", "", "string");
}

TEST_F(ValueParameterTest, SetValueFromRange)
{
    ParameterPtr range = factory::declareRange("range", 0, 100, 50, 1);
    ParameterPtr value = factory::declareValue("foo", 0);

    value->cloneDataFrom(*range);

    ASSERT_EQ(50, value->as<int>());
}




TEST_F(ValueParameterTest, SetValueFromAnotherType)
{
    ParameterPtr value = factory::declareValue("foo", 42.0);

    ASSERT_NEAR(42.0, value->as<double>(), 1e-5);

    ParameterPtr angle = factory::declareAngle("foo_angle", 0.0);
    ASSERT_NEAR(0.0, angle->as<double>(), 1e-5);
    angle->cloneDataFrom(*value);

    ASSERT_NEAR(42.0, angle->as<double>(), 1e-5);
}

TEST_F(ValueParameterTest, ValueChangedIsTriggered)
{
    ParameterPtr value = factory::declareValue("foo", 42.0);

    bool called = false;
    value->parameter_changed.connect([&](param::Parameter* p) { called = (p == value.get()); });

    ASSERT_NEAR(42.0, value->as<double>(), 1e-5);

    value->set(23.0);

    ASSERT_TRUE(called);
}

TEST_F(ValueParameterTest, NumberConversion)
{
    ParameterPtr value = factory::declareValue("foo", 42.0);

    ASSERT_NEAR(42.0, value->as<double>(), 1e-5);
    ASSERT_NEAR(42.0f, value->as<float>(), 1e-5);

    ASSERT_THROW(value->as<int>(), std::runtime_error);
}

