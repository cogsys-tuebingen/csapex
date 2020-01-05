#include <csapex/param/parameter.h>
#include <csapex/param/value_parameter.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/param/string_list_parameter.h>
#include <csapex/param/null_parameter.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/delegate.h>
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/utility/yaml.h>

#include <csapex_testing/csapex_test_case.h>

#include <unordered_map>
#include <typeindex>

using namespace csapex;
using namespace csapex::param;

class NullParameterTest : public CsApexTestCase
{
protected:
    NullParameterTest()
    {
    }
};

namespace csapex
{
namespace param
{
namespace factory
{
// nulls must never be used intenionally, declare the factory here
ParameterBuilder declareNull(const std::string& name)
{
    std::shared_ptr<NullParameter> result(new NullParameter(name, ParameterDescription()));
    return ParameterBuilder(std::move(result));
}
}  // namespace factory
}  // namespace param
}  // namespace csapex

TEST_F(NullParameterTest, HasState)
{
    EXPECT_FALSE(factory::declareNull("foo").build()->hasState());
}

TEST_F(NullParameterTest, StringConversion)
{
    EXPECT_STREQ("[foo: [null]]", factory::declareNull("foo").build()->toString().c_str());
}

TEST_F(NullParameterTest, GetThrows)
{
    ParameterPtr p = factory::declareNull("foo").build();
    EXPECT_ANY_THROW(p->as<int>());
}

TEST_F(NullParameterTest, Serialization)
{
    YAML::Node node;

    {
        ParameterPtr p = factory::declareNull("foo");
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
        ParameterPtr p = factory::declareNull("foo");
        ParameterSerializer::instance().serialize(*p, buffer);
    }

    {
        StreamablePtr s = ParameterSerializer::instance().deserialize(buffer);
        ASSERT_NE(nullptr, s);

        ParameterPtr p = std::dynamic_pointer_cast<Parameter>(s);
        ASSERT_NE(nullptr, p);

        NullParameter::Ptr sp = std::dynamic_pointer_cast<NullParameter>(p);
        ASSERT_NE(nullptr, sp);
    }
}

TEST_F(NullParameterTest, BinarySerialization)
{
    YAML::Node node;

    {
        ParameterPtr p = factory::declareNull("foo");
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
        ParameterPtr p = factory::declareNull("foo");
        ParameterSerializer::instance().serialize(*p, buffer);
    }

    {
        StreamablePtr s = ParameterSerializer::instance().deserialize(buffer);
        ASSERT_NE(nullptr, s);

        ParameterPtr p = std::dynamic_pointer_cast<Parameter>(s);
        ASSERT_NE(nullptr, p);

        NullParameter::Ptr sp = std::dynamic_pointer_cast<NullParameter>(p);
        ASSERT_NE(nullptr, sp);
    }
}
