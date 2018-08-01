#include <csapex/param/parameter.h>
#include <csapex/param/value_parameter.h>
#include <csapex/param/set_parameter.h>
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

std::map<std::string, int> encodings_i = { { "YUV", (int)YUV }, { "RGB", (int)RGB },   { "BGR", (int)BGR }, { "HSL", (int)HSL },
                                           { "HSV", (int)HSV }, { "MONO", (int)MONO }, { "LAB", (int)LAB }, { "LUV", (int)LUV } };
std::map<std::string, double> double_set = { { "1.5", 1.5 }, { "3.5", 3.5 } };
std::map<std::string, std::string> string_set = { { "1.5", "1.5" }, { "3.5", "3.5" } };


TEST_F(SetParameterTest, HasState)
{
    EXPECT_TRUE(factory::declareParameterSet("foo", encodings_i, (int)MONO).build()->hasState());
    EXPECT_TRUE(factory::declareParameterSet("foo", double_set, 1.5).build()->hasState());
    EXPECT_TRUE(factory::declareParameterSet("foo", string_set, std::string("1.5")).build()->hasState());
}

TEST_F(SetParameterTest, GetTexts)
{
    SetParameterPtr a = csapex::param::factory::declareParameterSet("input", encodings_i, (int)MONO).build<SetParameter>();
    auto texts = a->getSetTexts();
    ASSERT_EQ(encodings_i.size(), texts.size());

    auto it = encodings_i.begin();
    for(std::size_t i = 0, n = encodings_i.size(); i < n; ++i, ++it) {
        ASSERT_STREQ(it->first.c_str(), texts.at(i).c_str());
        ASSERT_STREQ(it->first.c_str(), a->getText(i).c_str());
    }
}

TEST_F(SetParameterTest, InvalidDefaultOptionThrows)
{
    ASSERT_ANY_THROW(factory::declareParameterSet("foo", encodings_i, 200));
    ASSERT_ANY_THROW(factory::declareParameterSet("foo", double_set, 200.0));
    ASSERT_ANY_THROW(factory::declareParameterSet("foo", string_set, std::string("200.0")));
}

TEST_F(SetParameterTest, StringConversion)
{
    {
        std::stringstream expected; expected << "[foo: [set: " << (int) MONO << "]]";
        EXPECT_STREQ(expected.str().c_str(), factory::declareParameterSet("foo", encodings_i, (int)MONO).build()->toString().c_str());
    }
    {
        std::stringstream expected; expected << "[foo: [set: " << 1.5 << "]]";
        EXPECT_STREQ(expected.str().c_str(), factory::declareParameterSet("foo", double_set, 1.5).build()->toString().c_str());
    }
    {
        std::stringstream expected; expected << "[foo: [set: " << "1.5" << "]]";
        EXPECT_STREQ(expected.str().c_str(), factory::declareParameterSet("foo", string_set, std::string("1.5")).build()->toString().c_str());
    }
}


TEST_F(SetParameterTest, CloningWorksForSet)
{
    {
        ParameterPtr a = csapex::param::factory::declareParameterSet("input", encodings_i, (int)MONO);
        ASSERT_EQ((int)MONO, a->as<int>());

        SetParameterPtr a_clone = a->cloneAs<SetParameter>();
        ASSERT_EQ((int)MONO, a_clone->as<int>());

        a_clone->setByName("RGB");
        ASSERT_EQ((int)RGB, a_clone->as<int>());
    }
    {
        ParameterPtr a = csapex::param::factory::declareParameterSet("input", double_set, 1.5);
        ASSERT_EQ(1.5, a->as<double>());

        SetParameterPtr a_clone = a->cloneAs<SetParameter>();
        ASSERT_EQ(1.5, a_clone->as<double>());

        a_clone->setByName("1.5");
        ASSERT_EQ(1.5, a_clone->as<double>());
    }
    {
        ParameterPtr a = csapex::param::factory::declareParameterSet("input", string_set, std::string("1.5"));
        ASSERT_STREQ("1.5", a->as<std::string>().c_str());

        ParameterPtr a_clone = a->cloneAs<Parameter>();
        ASSERT_STREQ("1.5", a_clone->as<std::string>().c_str());
    }
}


TEST_F(SetParameterTest, StringSet)
{
    {
        std::vector<std::string> string_vector = { "1.5", "3.5" };
        ParameterPtr a = csapex::param::factory::declareParameterStringSet("input", string_vector, std::string("1.5"));
        ASSERT_STREQ("1.5", a->as<std::string>().c_str());

        ParameterPtr a_clone = a->cloneAs<Parameter>();
        ASSERT_STREQ("1.5", a_clone->as<std::string>().c_str());

        SetParameterPtr a_set = a->cloneAs<SetParameter>();
        ASSERT_STREQ("1.5", a_set->as<std::string>().c_str());
        a_set->setByName("3.5");
        ASSERT_STREQ("3.5", a_set->as<std::string>().c_str());
    }
}

