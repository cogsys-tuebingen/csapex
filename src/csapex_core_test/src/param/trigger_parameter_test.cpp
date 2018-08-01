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

class TriggerParameterTest : public CsApexTestCase
{
protected:
    TriggerParameterTest()
    {
    }
};

TEST_F(TriggerParameterTest, HasState)
{
    EXPECT_FALSE(factory::declareTrigger("foo").build()->hasState());
}

TEST_F(TriggerParameterTest, StringConversion)
{
    EXPECT_STREQ("[foo: [trigger]]", factory::declareTrigger("foo").build()->toString().c_str());
}


TEST_F(TriggerParameterTest, Serialization)
{
    YAML::Node node;

    {
        ParameterPtr p = factory::declareTrigger("foo");
        p->serialize_yaml(node);
        EXPECT_STREQ(p->name().c_str(), node["name"].as<std::string>().c_str());
        EXPECT_STREQ(p->getParameterType().c_str(), node["type"].as<std::string>().c_str());
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


TEST_F(TriggerParameterTest, BinarySerialization)
{
    YAML::Node node;

    {
        ParameterPtr p = factory::declareTrigger("foo");
        p->serialize_yaml(node);
        EXPECT_STREQ(p->name().c_str(), node["name"].as<std::string>().c_str());
        EXPECT_STREQ(p->getParameterType().c_str(), node["type"].as<std::string>().c_str());
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

TEST_F(TriggerParameterTest, SetValueFromTriggerThrows)
{
    ParameterPtr trigger = factory::declareTrigger("range");
    ParameterPtr value = factory::declareValue("foo", 0);

    ASSERT_ANY_THROW(value->cloneDataFrom(*trigger));
}


TEST_F(TriggerParameterTest, SettingTriggerCausesTrigger)
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
