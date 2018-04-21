#include <csapex/param/parameter.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/delegate.h>

#include <unordered_map>
#include <typeindex>

#include "gtest/gtest.h"

using namespace csapex;
using namespace csapex::param;

class ParameterTest : public ::testing::Test
{
protected:
    ParameterTest()
    {
    }
};


TEST_F(ParameterTest, TestGenericSetFrom)
{
    ParameterPtr a = ParameterFactory::declareValue("foo", 42);
    ParameterPtr b = ParameterFactory::declareValue("bar", 23);
    ParameterPtr c = ParameterFactory::declareValue("baz", 5);

    ASSERT_EQ(42, a->as<int>());
    ASSERT_EQ(23, b->as<int>());
    ASSERT_EQ(5, c->as<int>());

    a->setValueFrom(*b);
    ASSERT_EQ(a->as<int>(), b->as<int>());

    *a = *c;
    ASSERT_EQ(a->as<int>(), c->as<int>());
}

TEST_F(ParameterTest, CloningWorksForInt)
{
    ParameterPtr a = ParameterFactory::declareValue("foo", 42);

    ASSERT_EQ(42, a->as<int>());

    ParameterPtr a_clone = a->cloneAs<Parameter>();
    ASSERT_EQ(42, a_clone->as<int>());
}

TEST_F(ParameterTest, CloningWorksForSet)
{
    enum ColorSpace {YUV, RGB, BGR, HSL, HSV, MONO, LAB, LCh, LUV};

    std::map<std::string, int> encodings = {
        {"YUV", (int) YUV},
        {"RGB", (int) RGB},
        {"BGR", (int) BGR},
        {"HSL", (int) HSL},
        {"HSV", (int) HSV},
        {"MONO",(int) MONO},
        {"LAB", (int) LAB},
        {"LUV" ,(int) LUV}
    };

    ParameterPtr a = csapex::param::ParameterFactory::declareParameterSet("input", encodings, (int) MONO);
    ASSERT_EQ((int) MONO, a->as<int>());

    ParameterPtr a_clone = a->cloneAs<Parameter>();
    ASSERT_EQ((int) MONO, a_clone->as<int>());
}
