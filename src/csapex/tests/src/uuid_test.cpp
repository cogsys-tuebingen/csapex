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
    UUID uuid1 = uuid_provider->makeUUID("foo");
    ASSERT_EQ("foo", uuid1.getFullName());

    try {
        uuid_provider->makeUUID("foo");
        FAIL();

    } catch(const std::exception& e) {
        SUCCEED();
    }

    UUID uuid2 = uuid_provider->generateUUID("bar");
    ASSERT_EQ("bar_0", uuid2.getFullName());

    ASSERT_EQ(1, uuid_provider->getUUIDMap()["bar"]);
}

TEST_F(UUIDTest, UUIDsAreUnique)
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

TEST_F(UUIDTest, MakeTypedUUIDWithIntegerId)
{
    UUID parent = uuid_provider->generateUUID("foo");
    UUID bar = uuid_provider->makeTypedUUID(parent, "bar", 23);

    ASSERT_EQ("foo_0:|:bar_23", bar.getFullName());
}

TEST_F(UUIDTest, MakeTypedUUIDWithStringId)
{
    UUID parent = uuid_provider->generateUUID("foo");
    UUID bar = uuid_provider->makeTypedUUID(parent, "bar", "baz");

    ASSERT_EQ("foo_0:|:bar_baz", bar.getFullName());
}


TEST_F(UUIDTest, MakeTypedUUIDWithoutParent)
{
    UUID bar = uuid_provider->makeTypedUUID(UUID::NONE, "bar", 23);

    ASSERT_EQ(UUID::NONE, bar);
}


TEST_F(UUIDTest, MakeForcedTypedUUIDWithoutParent)
{
    UUID bar = uuid_provider->makeTypedUUID_forced(UUID::NONE, "bar", 23);

    ASSERT_EQ(UUID::NONE, bar);
}


TEST_F(UUIDTest, GenerateTypedUUID)
{
    UUID parent = uuid_provider->generateUUID("foo");
    UUID bar = uuid_provider->generateTypedUUID(parent, "bar");

    ASSERT_EQ("foo_0:|:bar_0", bar.getFullName());
}

TEST_F(UUIDTest, GenerateTypedUUIDWithoutParent)
{
    UUID bar = uuid_provider->generateTypedUUID(UUID::NONE, "bar");

    ASSERT_EQ(UUID::NONE, bar);
}

TEST_F(UUIDTest, AbsoluteUUIDSAreAbsolute)
{
    UUIDProviderPtr parent = std::make_shared<UUIDProvider>();
    UUID parent_id = parent->generateUUID("foo");

    UUIDProviderPtr child = std::make_shared<UUIDProvider>();
    child->setParent(parent, AUUID(parent_id));
    UUID child_id = child->generateUUID("bar");

    ASSERT_EQ("bar_0", child_id.getFullName());
    ASSERT_EQ("foo_0:|:bar_0", child_id.getAbsoluteUUID().getFullName());
    ASSERT_EQ("foo_0:|:bar_0", child_id.getAbsoluteUUID().getAbsoluteUUID().getAbsoluteUUID().getFullName());
}

TEST_F(UUIDTest, UUIDsCanBeReshaped)
{
    UUID foo = uuid_provider->generateUUID("foo");
    UUID bar = uuid_provider->generateDerivedUUID(foo, "bar");
    UUID baz = uuid_provider->generateDerivedUUID(bar, "baz");

    ASSERT_EQ("foo_0:|:bar_0:|:baz_0", baz.getFullName());

    ASSERT_EQ(3, baz.reshape(3).depth());
    ASSERT_EQ("foo_0:|:bar_0:|:baz_0", baz.reshape(3).getFullName());

    ASSERT_EQ(2, baz.reshape(2).depth());
    ASSERT_EQ("bar_0:|:baz_0", baz.reshape(2).getFullName());

    ASSERT_EQ(1, baz.reshape(1).depth());
    ASSERT_EQ("baz_0", baz.reshape(1).getFullName());

    ASSERT_EQ(0, baz.reshape(0).depth());
    ASSERT_TRUE(baz.reshape(0).empty());

    ASSERT_THROW(baz.reshape(4), std::invalid_argument);
    ASSERT_THROW(baz.reshape(1000), std::invalid_argument);
}


//test reshaping thoroughly
//refactor other methods to use reshape
//implement reshape more efficiently
