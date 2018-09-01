#include "gtest/gtest.h"

#include <csapex/utility/type.h>

using namespace csapex;

class TypeTest : public ::testing::Test
{
protected:
    TypeTest()
    {
    }

    virtual ~TypeTest()
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp() override
    {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    virtual void TearDown() override
    {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }
};

namespace A
{
namespace B
{
namespace C
{
class Test
{
};
}  // namespace C
}  // namespace B
}  // namespace A

TEST_F(TypeTest, FullNamespaceWorks)
{
    ASSERT_STREQ("A::B::C::Test", type2name(typeid(A::B::C::Test)).c_str());

    using A::B::C::Test;
    ASSERT_STREQ("A::B::C::Test", type2name(typeid(Test)).c_str());
}

TEST_F(TypeTest, AllNamespacesAreRemoved)
{
    ASSERT_STREQ("Test", type2nameWithoutNamespace(typeid(A::B::C::Test)).c_str());

    using A::B::C::Test;
    ASSERT_STREQ("Test", type2nameWithoutNamespace(typeid(Test)).c_str());
}
