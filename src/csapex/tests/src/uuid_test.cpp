#include "gtest/gtest.h"

#include <csapex/utility/uuid_provider.h>

using namespace csapex;

class UUIDTest : public ::testing::Test
{
protected:
    UUIDTest()
        : uuid_provider(std::make_shared<UUIDProvider>())
    {
    }

    virtual ~UUIDTest() {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp() override {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    virtual void TearDown() override {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    UUIDProviderPtr uuid_provider;
};

TEST_F(UUIDTest, UUIDsAreGeneratedCorrectly)
{
    UUID uuid1 = uuid_provider->generateUUID("foo");
    ASSERT_EQ("foo_0", uuid1.getFullName());

    UUID uuid2 = uuid_provider->generateUUID("foo");
    ASSERT_EQ("foo_1", uuid2.getFullName());
}


TEST_F(UUIDTest, UUIDsAreOnlyUniqueAcrossProviders)
{
    auto uuid_provider = std::make_shared<UUIDProvider>();

    UUID uuid1 = uuid_provider->generateUUID("foo");
    ASSERT_EQ("foo_0", uuid1.getFullName());

    UUID uuid2 = uuid_provider->generateUUID("foo");
    ASSERT_EQ("foo_1", uuid2.getFullName());

    UUIDProviderPtr uuid_provider_2 = std::make_shared<UUIDProvider>();
    UUID uuid1_2 = uuid_provider_2->generateUUID("foo");
    ASSERT_EQ("foo_0", uuid1_2.getFullName());

    UUID uuid2_2 = uuid_provider_2->generateUUID("foo");
    ASSERT_EQ("foo_1", uuid2_2.getFullName());
}


TEST_F(UUIDTest, UUIDsCanBeChildren)
{
    UUID parent = uuid_provider->generateUUID("foo");
    ASSERT_EQ("foo_0", parent.getFullName());
    UUID bar1 = uuid_provider->generateDerivedUUID(parent, "bar");
    ASSERT_EQ("foo_0:|:bar_0", bar1.getFullName());
    UUID bar2 = uuid_provider->generateDerivedUUID(parent, "bar");
    ASSERT_EQ("foo_0:|:bar_1", bar2.getFullName());
}

TEST_F(UUIDTest, UUIDsCanBeNested)
{
    UUID foo = uuid_provider->generateUUID("foo");
    UUID bar = uuid_provider->generateDerivedUUID(foo, "bar");
    UUID baz = uuid_provider->generateDerivedUUID(bar, "baz");

    ASSERT_EQ("foo_0:|:bar_0:|:baz_0", baz.getFullName());

    UUID full = UUIDProvider::makeUUID_without_parent("foo_0:|:bar_0:|:baz_0");

    ASSERT_EQ("foo_0", full.rootUUID().getFullName());
    ASSERT_EQ("bar_0:|:baz_0", full.nestedUUID().getFullName());
}


TEST_F(UUIDTest, UUIDsCanBeDeconstructed)
{
    UUID foo = uuid_provider->generateUUID("foo");
    UUID bar = uuid_provider->generateDerivedUUID(foo, "bar");
    UUID baz = uuid_provider->generateDerivedUUID(bar, "baz");

    ASSERT_EQ("foo_0:|:bar_0:|:baz_0", baz.getFullName());
    ASSERT_EQ("foo_0:|:bar_0", baz.parentUUID().getFullName());
    ASSERT_EQ("foo_0", baz.parentUUID().parentUUID().getFullName());

    ASSERT_EQ("foo_0", foo.id());
    ASSERT_EQ("bar_0", bar.id());
    ASSERT_EQ("baz_0", baz.id());

    ASSERT_EQ("foo_0", foo.rootUUID());
    ASSERT_EQ("foo_0", bar.rootUUID());
    ASSERT_EQ("foo_0", baz.rootUUID());
}
