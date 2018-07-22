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

class SetParameterTest : public CsApexTestCase
{
protected:
    SetParameterTest()
    {
    }
};


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


TEST_F(SetParameterTest, HasState)
{
    EXPECT_TRUE(factory::declareParameterSet("foo", encodings, (int)MONO).build()->hasState());
}

TEST_F(SetParameterTest, StringConversion)
{
    std::stringstream expected; expected << "[foo: [set: " << (int) MONO << "]]";
    EXPECT_STREQ(expected.str().c_str(), factory::declareParameterSet("foo", encodings, (int)MONO).build()->toString().c_str());
}


TEST_F(SetParameterTest, CloningWorksForSet)
{
    ParameterPtr a = csapex::param::factory::declareParameterSet("input", encodings, (int)MONO);
    ASSERT_EQ((int)MONO, a->as<int>());

    ParameterPtr a_clone = a->cloneAs<Parameter>();
    ASSERT_EQ((int)MONO, a_clone->as<int>());
}
