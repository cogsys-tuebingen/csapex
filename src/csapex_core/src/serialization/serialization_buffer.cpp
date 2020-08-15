/// HEADER
#include <csapex/serialization/io/std_io.h>

/// PROJECT
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>
#include <csapex/model/tracing_type.h>
#include <csapex/model/connection_description.h>
#include <csapex/model/connector_description.h>
#include <csapex/model/connector_type.h>
#include <csapex/model/error_state.h>
#include <csapex/model/execution_state.h>
#include <csapex/model/node_characteristics.h>
#include <csapex/model/node_state.h>
#include <csapex/model/token_data.h>
#include <csapex/param/parameter.h>
#include <csapex/profiling/interval.h>
#include <csapex/serialization/packet_serializer.h>
#include <csapex/serialization/snippet.h>
#include <csapex/serialization/streamable.h>
#include <csapex/model/notification.h>
#include <csapex/utility/yaml.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

bool SerializationBuffer::initialized_ = false;
std::map<std::type_index, std::function<void(SerializationBuffer& buffer, const std::any& a)>> SerializationBuffer::any_serializer;
std::map<uint8_t, std::function<void(const SerializationBuffer& buffer, std::any& a)>> SerializationBuffer::any_deserializer;

SerializationBuffer::SerializationBuffer() : pos(HEADER_LENGTH)
{
    // the header is always 4 byte
    insert(end(), HEADER_LENGTH, 0);

    init();
}

SerializationBuffer::SerializationBuffer(const std::vector<uint8_t>& copy, bool insert_header) : pos(HEADER_LENGTH)
{
    if (insert_header) {
        // the header is always 4 byte
        insert(end(), HEADER_LENGTH, 0);
        insert(end(), copy.begin(), copy.end());

    } else {
        assign(copy.begin(), copy.end());
    }

    init();
}

SerializationBuffer::SerializationBuffer(const uint8_t* raw_data, const std::size_t length, bool insert_header) : pos(HEADER_LENGTH)
{
    if (insert_header) {
        // the header is always 4 byte
        insert(end(), HEADER_LENGTH, 0);
        insert(end(), raw_data, raw_data + length);

    } else {
        assign(raw_data, raw_data + length);
    }

    init();
}

void SerializationBuffer::init()
{
    if (!initialized_) {
#define ADD(...)                                                                                                                                                                                       \
    any_serializer[std::type_index(typeid(__VA_ARGS__))] = serializer;                                                                                                                                 \
    any_deserializer[id] = deserializer;                                                                                                                                                               \
    }                                                                                                                                                                                                  \
    ++id
#define ADD_ANY_TYPE_IMPL(...)                                                                                                                                                                         \
    {                                                                                                                                                                                                  \
        auto serializer = [id](SerializationBuffer& buffer, const std::any& any) {                                                                                                                   \
            buffer << ((uint8_t)id);                                                                                                                                                                   \
            buffer << (std::any_cast<__VA_ARGS__>(any));                                                                                                                                             \
        };                                                                                                                                                                                             \
        auto deserializer = [](const SerializationBuffer& buffer, std::any& any) {                                                                                                                 \
            __VA_ARGS__ v;                                                                                                                                                                             \
            buffer >> (v);                                                                                                                                                                             \
            any = v;                                                                                                                                                                                   \
        };                                                                                                                                                                                             \
        ADD(__VA_ARGS__)

#define ADD_ANY_TYPE(...)                                                                                                                                                                              \
    ADD_ANY_TYPE_IMPL(__VA_ARGS__);                                                                                                                                                                    \
    ADD_ANY_TYPE_IMPL(std::vector<__VA_ARGS__>)

#define ADD_ANY_TYPE_1(P1, GET_P1, ...)                                                                                                                                                                \
    {                                                                                                                                                                                                  \
        auto serializer = [id](SerializationBuffer& buffer, const std::any& any) {                                                                                                                   \
            buffer << ((uint8_t)id);                                                                                                                                                                   \
            buffer << (std::any_cast<__VA_ARGS__>(any)).GET_P1;                                                                                                                                      \
            buffer << (std::any_cast<__VA_ARGS__>(any));                                                                                                                                             \
        };                                                                                                                                                                                             \
        auto deserializer = [](const SerializationBuffer& buffer, std::any& any) {                                                                                                                 \
            P1 p1;                                                                                                                                                                                     \
            buffer >> (p1);                                                                                                                                                                            \
            __VA_ARGS__ v(p1);                                                                                                                                                                         \
            buffer >> (v);                                                                                                                                                                             \
            any = v;                                                                                                                                                                                   \
        };                                                                                                                                                                                             \
        ADD(__VA_ARGS__)

#define ADD_ANY_TYPE_1PC(P1, GET_P1, ...)                                                                                                                                                              \
    {                                                                                                                                                                                                  \
        auto serializer = [id](SerializationBuffer& buffer, const std::any& any) {                                                                                                                   \
            buffer << ((uint8_t)id);                                                                                                                                                                   \
            buffer << (std::any_cast<std::shared_ptr<__VA_ARGS__ const>>(any))->GET_P1;                                                                                                              \
            buffer << *(std::any_cast<std::shared_ptr<__VA_ARGS__ const>>(any));                                                                                                                     \
        };                                                                                                                                                                                             \
        auto deserializer = [](const SerializationBuffer& buffer, std::any& any) {                                                                                                                 \
            P1 p1;                                                                                                                                                                                     \
            buffer >> (p1);                                                                                                                                                                            \
            auto v = std::make_shared<__VA_ARGS__>(p1);                                                                                                                                                \
            buffer >> (*v);                                                                                                                                                                            \
            any = std::shared_ptr<__VA_ARGS__ const>(v);                                                                                                                                               \
        };                                                                                                                                                                                             \
        ADD(std::shared_ptr<__VA_ARGS__ const>)

        int id = 1;
        ADD_ANY_TYPE(int);
        ADD_ANY_TYPE(double);
        ADD_ANY_TYPE(bool);
        ADD_ANY_TYPE(std::string);
        ADD_ANY_TYPE(std::vector<std::string>);
        ADD_ANY_TYPE(long);
        ADD_ANY_TYPE(UUID);
        ADD_ANY_TYPE(AUUID);
        ADD_ANY_TYPE(std::pair<std::string, bool>);
        ADD_ANY_TYPE(std::pair<std::string, bool>);
        ADD_ANY_TYPE(std::pair<int, int>);
        ADD_ANY_TYPE(std::pair<std::string, bool>);
        ADD_ANY_TYPE(std::pair<double, double>);
        ADD_ANY_TYPE(ConnectorType);
        ADD_ANY_TYPE(YAML::Node);
        //        ADD_ANY_TYPE_1(std::string, typeName(), TokenData);
        ADD_ANY_TYPE(TokenDataConstPtr);
        ADD_ANY_TYPE(SnippetPtr);
        ADD_ANY_TYPE(NodeCharacteristics);
        ADD_ANY_TYPE(ConnectorDescription);
        ADD_ANY_TYPE(ConnectionDescription);
        ADD_ANY_TYPE(ExecutionState);
        ADD_ANY_TYPE(Notification);
        ADD_ANY_TYPE(Fulcrum);
        ADD_ANY_TYPE(NodeStatePtr);
        ADD_ANY_TYPE(param::ParameterPtr);
        ADD_ANY_TYPE(TracingType);
        ADD_ANY_TYPE(ErrorState::ErrorLevel);
        ADD_ANY_TYPE_1PC(std::string, name(), Interval);

        initialized_ = true;
    }
}

void SerializationBuffer::finalize()
{
    uint32_t length = size();
    apex_assert_lte_hard(length, std::numeric_limits<uint32_t>::max());

    std::size_t nbytes = sizeof(uint32_t);
    for (std::size_t byte = 0; byte < nbytes; ++byte) {
        uint8_t part = (length >> (byte * 8)) & 0xFF;
        at(byte) = part;
    }
}

void SerializationBuffer::seek(uint32_t p) const
{
    pos = p;
    apex_assert_lte(pos, size());
}
void SerializationBuffer::advance(uint32_t p) const
{
    pos += p;
    apex_assert_lte(pos, size());
}

void SerializationBuffer::rewind() const
{
    pos = HEADER_LENGTH;
}

uint32_t SerializationBuffer::getPos() const
{
    return pos;
}

std::string SerializationBuffer::toString() const
{
    std::stringstream res;
    for (std::size_t i = 0; i < size(); ++i) {
        if ((i % 8) == 0) {
            res << std::hex << i << ":\t\t" << std::dec;
        }

        res << (int)at(i);

        if ((i % 8) != 7) {
            res << '\t';
        } else {
            res << '\n';
        }
    }
    return res.str();
}

void SerializationBuffer::write(const Streamable& i)
{
    bool null = false;
    operator<<(null);
    PacketSerializer::instance().serialize(i, *this);
}
void SerializationBuffer::write(const StreamableConstPtr& i)
{
    bool null = (i == nullptr);
    operator<<(null);
    if (!null) {
        PacketSerializer::instance().serialize(*i, *this);
    }
}

StreamablePtr SerializationBuffer::read() const
{
    bool null;
    operator>>(null);
    if (!null) {
        return PacketSerializer::instance().deserialize(*this);
    }
    return nullptr;
}

void SerializationBuffer::writeRaw(const char* data, const std::size_t length)
{
    reserve(size() + length);
    std::copy(data, data + length, std::back_inserter(*this));
}

void SerializationBuffer::writeRaw(const uint8_t* data, const std::size_t length)
{
    reserve(size() + length);
    std::copy(data, data + length, std::back_inserter(*this));
}

void SerializationBuffer::readRaw(char* data, const std::size_t length) const
{
    const char* start = reinterpret_cast<const char*>(&at(pos));
    std::copy(start, start + length, data);
    pos += length;
}

void SerializationBuffer::readRaw(uint8_t* data, const std::size_t length) const
{
    const uint8_t* start = &at(pos);
    std::copy(start, start + length, data);
    pos += length;
}

SerializationBuffer& SerializationBuffer::writeAny(const std::any& any)
{
    auto fn = any_serializer.find(any.type());
    if (fn != any_serializer.end()) {
        fn->second(*this, any);
    } else {
        if (any_has_value(any)) {
            std::cerr << "cannot serialize std::any containing " << type2name(any.type()) << std::endl;
        }
        operator<<((uint8_t)0);
    }

    return *this;
}

const SerializationBuffer& SerializationBuffer::readAny(std::any& any) const
{
    uint8_t type;
    operator>>(type);

    if (type == 0) {
        // empty std::any
        return *this;
    }

    auto fn = any_deserializer.find(type);
    if (fn != any_deserializer.end()) {
        fn->second(*this, any);
    } else {
        std::cerr << "cannot deserialize std::any with type " << (int)type << std::endl;
    }

    return *this;
}

#if G_BYTE_ORDER == G_LITTLE_ENDIAN
union _GFloatIEEE754
{
    float v_float;
    struct
    {
        uint mantissa : 23;
        uint biased_exponent : 8;
        uint sign : 1;
    } mpn;
};
union _GDoubleIEEE754
{
    double v_double;
    struct
    {
        uint mantissa_low : 32;
        uint mantissa_high : 20;
        uint biased_exponent : 11;
        uint sign : 1;
    } mpn;
};
#elif G_BYTE_ORDER == G_BIG_ENDIAN
union _GFloatIEEE754
{
    float v_float;
    struct
    {
        uint sign : 1;
        uint biased_exponent : 8;
        uint mantissa : 23;
    } mpn;
};
union _GDoubleIEEE754
{
    double v_double;
    struct
    {
        uint sign : 1;
        uint biased_exponent : 11;
        uint mantissa_high : 20;
        uint mantissa_low : 32;
    } mpn;
};
#else /* !G_LITTLE_ENDIAN && !G_BIG_ENDIAN */
#error unknown ENDIAN type
#endif /* !G_LITTLE_ENDIAN && !G_BIG_ENDIAN */

SerializationBuffer& SerializationBuffer::operator<<(float f)
{
    _GFloatIEEE754 ieee;
    ieee.v_float = f;

    /***
     * sign  | mantissa    |  exponent
     *   1         23             8
     *   1   | 2 - 24      | 25 - 32
     */
    uint8_t bytes[4];
    // memset(&bytes[0], 0, 4);
    bytes[0] = ieee.mpn.sign << 7;
    bytes[0] |= (ieee.mpn.mantissa >> (23 - 7));
    bytes[1] = (ieee.mpn.mantissa >> (23 - 7 - 8)) & 0xFF;
    bytes[2] = (ieee.mpn.mantissa >> (23 - 7 - 16)) & 0xFF;
    bytes[3] = ieee.mpn.biased_exponent;

    for (uint8_t i = 0; i < 4; ++i) {
        push_back(bytes[i]);
    }

    return *this;
}

const SerializationBuffer& SerializationBuffer::operator>>(float& f) const
{
    uint8_t bytes[4];
    for (uint8_t i = 0; i < 4; ++i) {
        bytes[i] = at(pos++);
    }

    _GFloatIEEE754 ieee;

    ieee.mpn.sign = (bytes[0] & (1 << 7)) ? 1 : 0;
    ieee.mpn.mantissa = ((bytes[0] & (~(1 << 7))) << (23 - 7)) | (bytes[1] << (23 - 7 - 8)) | (bytes[2] << (23 - 7 - 16));
    ieee.mpn.biased_exponent = bytes[3];

    f = ieee.v_float;

    return *this;
}

// DOUBLES
SerializationBuffer& SerializationBuffer::operator<<(double d)
{
    _GDoubleIEEE754 ieee;
    ieee.v_double = d;

    /***
     * sign  | exponent   |  mantissa_high   | mantissa_low
     *   1         11             20         |   32
     *   1   | 2 - 12         | 13 - 32      |   32
     */
    uint8_t bytes[8];
    // memset(&bytes[0], 0, 4);
    bytes[0] = ieee.mpn.sign << 7;
    bytes[0] |= (ieee.mpn.biased_exponent >> (11 - 7));
    bytes[1] = (ieee.mpn.biased_exponent << 4);
    bytes[1] |= (ieee.mpn.mantissa_high >> (20 - 4)) & 0xFF;
    bytes[2] = (ieee.mpn.mantissa_high >> (20 - 4 - 8)) & 0xFF;
    bytes[3] = (ieee.mpn.mantissa_high >> (20 - 4 - 16)) & 0xFF;
    bytes[4] = (ieee.mpn.mantissa_low >> (3 * 8)) & 0xFF;
    bytes[5] = (ieee.mpn.mantissa_low >> (2 * 8)) & 0xFF;
    bytes[6] = (ieee.mpn.mantissa_low >> (1 * 8)) & 0xFF;
    bytes[7] = (ieee.mpn.mantissa_low >> (0 * 8)) & 0xFF;

    for (uint8_t i = 0; i < 8; ++i) {
        push_back(bytes[i]);
    }

    return *this;
}

const SerializationBuffer& SerializationBuffer::operator>>(double& d) const
{
    uint8_t bytes[8];
    for (uint8_t i = 0; i < 8; ++i) {
        bytes[i] = at(pos++);
    }

    _GDoubleIEEE754 ieee;

    ieee.mpn.sign = (bytes[0] & (1 << 7)) ? 1 : 0;
    ieee.mpn.biased_exponent = ((bytes[0] & (~(1 << 7))) << (11 - 7)) | ((bytes[1] & (0xF0)) >> 4);
    ieee.mpn.mantissa_high = ((bytes[1] & (~(1 << 7))) << (20 - 4)) | (bytes[2] << (20 - 4 - 8)) | (bytes[3] << (20 - 4 - 16));

    ieee.mpn.mantissa_low = (bytes[4] << (3 * 8)) | (bytes[5] << (2 * 8)) | (bytes[6] << (1 * 8)) | (bytes[7] << (0 * 8));

    d = ieee.v_double;

    return *this;
}

// YAML
SerializationBuffer& SerializationBuffer::operator<<(const YAML::Node& node)
{
    std::stringstream ss;
    ss << node;
    *this << ss;
    return *this;
}

const SerializationBuffer& SerializationBuffer::operator>>(YAML::Node& node) const
{
    std::stringstream ss;
    *this >> ss;
    node = YAML::Load(ss);
    return *this;
}
