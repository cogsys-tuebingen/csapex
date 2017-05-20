#include "gtest/gtest.h"

#include <csapex/msg/message_template.hpp>
#include <csapex/utility/register_msg.h>
#include <yaml-cpp/yaml.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/msg/io.h>

#include <bitset>

using namespace csapex;
using namespace connection_types;

class BinarySerializationTest : public ::testing::Test
{
protected:
    BinarySerializationTest()
    {
    }

    virtual ~BinarySerializationTest() {
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

};

TEST_F(BinarySerializationTest, TestUInt8)
{
    std::size_t STEPS = 64;

    for(uint8_t step = 0;
        step < STEPS;
        ++step)
    {
        uint8_t i = std::numeric_limits<uint8_t>::min() +
                (step  / (double) STEPS) * (std::numeric_limits<uint8_t>::max() - std::numeric_limits<uint8_t>::min());
        SerializationBuffer buffer;
        buffer << i;
        uint8_t value;
        buffer >> value;

        ASSERT_EQ(i, value);
    }
}


TEST_F(BinarySerializationTest, TestInt32)
{
    std::size_t STEPS = 64;

    for(uint8_t step = 0;
        step < STEPS;
        ++step)
    {
        uint32_t i = std::numeric_limits<int32_t>::min() +
                (step  / (double) STEPS) * (std::numeric_limits<int32_t>::max() - std::numeric_limits<int32_t>::min());
        SerializationBuffer buffer;
        buffer << i;
        int32_t value;
        buffer >> value;

        ASSERT_EQ(i, value);
    }
}


TEST_F(BinarySerializationTest, TestUInt16)
{
    std::size_t STEPS = 64;

    for(uint8_t step = 0;
        step < STEPS;
        ++step)
    {
        uint16_t i = std::numeric_limits<uint16_t>::min() +
                (step  / (double) STEPS) * (std::numeric_limits<uint16_t>::max() - std::numeric_limits<uint16_t>::min());
        SerializationBuffer buffer;
        buffer << i;
        uint16_t value;
        buffer >> value;

        ASSERT_EQ(i, value);
    }
}




TEST_F(BinarySerializationTest, TestUInt32)
{
    std::size_t STEPS = 64;

    for(uint8_t step = 0;
        step < STEPS;
        ++step)
    {
        uint32_t i = std::numeric_limits<uint32_t>::min() +
                (step  / (double) STEPS) * (std::numeric_limits<uint32_t>::max() - std::numeric_limits<uint32_t>::min());
        SerializationBuffer buffer;
        buffer << i;
        uint32_t value;
        buffer >> value;

        ASSERT_EQ(i, value);
    }
}


TEST_F(BinarySerializationTest, TestFloat)
{
    std::size_t STEPS = 64;

    for(uint8_t step = 0;
        step < STEPS;
        ++step)
    {
        float f = std::numeric_limits<float>::min() +
                (step  / (double) STEPS) * (std::numeric_limits<float>::max() - std::numeric_limits<float>::min());
        SerializationBuffer buffer;
        buffer << f;
        float value;
        buffer >> value;

        ASSERT_NEAR(f, value, 1e-5);

        buffer << -f;
        buffer >> value;

        ASSERT_NEAR(-f, value, 1e-5);
    }
}

TEST_F(BinarySerializationTest, TestDouble)
{
    std::size_t STEPS = 64;

    for(uint8_t step = 0;
        step < STEPS;
        ++step)
    {
        double d = std::numeric_limits<double>::min() +
                (step  / (double) STEPS) * (std::numeric_limits<double>::max() - std::numeric_limits<double>::min());
        SerializationBuffer buffer;
        buffer << d;
        double value;
        buffer >> value;

        ASSERT_NEAR(d, value, 1e-5);

        buffer << -d;
        buffer >> value;

        ASSERT_NEAR(-d, value, 1e-5);
    }
}

TEST_F(BinarySerializationTest, TestDoubleNAN)
{
    SerializationBuffer buffer;
    buffer << std::numeric_limits<double>::quiet_NaN();
    double value;
    buffer >> value;

    ASSERT_TRUE(std::isnan(value));
}

TEST_F(BinarySerializationTest, TestFloatNAN)
{
    SerializationBuffer buffer;
    buffer << std::numeric_limits<float>::quiet_NaN();
    float value;
    buffer >> value;

    ASSERT_TRUE(std::isnan(value));
}


TEST_F(BinarySerializationTest, TestDoubleINF)
{
    SerializationBuffer buffer;
    buffer << std::numeric_limits<double>::infinity();
    double value;
    buffer >> value;

    ASSERT_TRUE(std::isinf(value));
}

TEST_F(BinarySerializationTest, TestFloatINF)
{
    SerializationBuffer buffer;
    buffer << std::numeric_limits<float>::infinity();
    float value;
    buffer >> value;

    ASSERT_TRUE(std::isinf(value));
}


TEST_F(BinarySerializationTest, TestDoubleNegINF)
{
    SerializationBuffer buffer;
    buffer << -std::numeric_limits<double>::infinity();
    double value;
    buffer >> value;

    ASSERT_TRUE(std::isinf(value));
    ASSERT_EQ(-std::numeric_limits<double>::infinity(), value);
}

TEST_F(BinarySerializationTest, TestFloatNegINF)
{
    SerializationBuffer buffer;
    buffer << -std::numeric_limits<float>::infinity();
    float value;
    buffer >> value;

    ASSERT_TRUE(std::isinf(value));
    ASSERT_EQ(-std::numeric_limits<float>::infinity(), value);
}

TEST_F(BinarySerializationTest, TestBoolTrue)
{
    SerializationBuffer buffer;
    buffer << true;
    bool value;
    buffer >> value;

    ASSERT_EQ(true, value);
}


TEST_F(BinarySerializationTest, TestBoolFalse)
{
    SerializationBuffer buffer;
    buffer << false;
    bool value;
    buffer >> value;

    ASSERT_EQ(false, value);
}

TEST_F(BinarySerializationTest, TestString)
{
    std::string str("foo bar baz");

    SerializationBuffer buffer;
    buffer << str;
    std::string value;
    buffer >> value;

    ASSERT_EQ(str, value);
}

TEST_F(BinarySerializationTest, TestEmptyString)
{
    std::string str("");

    SerializationBuffer buffer;
    buffer << str;
    std::string value;
    buffer >> value;

    ASSERT_EQ(str, value);
}
