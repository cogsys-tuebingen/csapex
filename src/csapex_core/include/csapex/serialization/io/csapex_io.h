#ifndef CSAPEX_IO_H
#define CSAPEX_IO_H

/// COMPONENT
#include <csapex/serialization/serialization_buffer.h>

/// PROJECT
#include <csapex/utility/utility_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/utility/data_traits.hpp>

namespace csapex
{
// base
SerializationBuffer& operator<<(SerializationBuffer& data, const Serializable& s);
const SerializationBuffer& operator>>(const SerializationBuffer& data, Serializable& s);

SerializationBuffer& operator<<(SerializationBuffer& data, const SemanticVersion& s);
const SerializationBuffer& operator>>(const SerializationBuffer& data, SemanticVersion& s);

// UUID
SerializationBuffer& operator<<(SerializationBuffer& data, const UUID& s);
const SerializationBuffer& operator>>(const SerializationBuffer& data, UUID& s);

// std overloads
template <typename S, typename std::enable_if<std::is_base_of<Serializable, S>::value, int>::type = 0>
SerializationBuffer& operator<<(SerializationBuffer& data, const std::vector<S>& s)
{
    apex_assert_lt_hard(s.size(), std::numeric_limits<uint8_t>::max());
    data << (static_cast<uint8_t>(s.size()));
    for (const S& elem : s) {
        // disambiguate possible overloads for serializable objects
        data << static_cast<const Serializable&>(elem);
    }
    return data;
}

template <typename S, typename std::enable_if<std::is_integral<S>::value && std::is_base_of<Serializable, S>::value, int>::type = 0>
const SerializationBuffer& operator>>(const SerializationBuffer& data, std::vector<S>& s)
{
    uint8_t len;
    data >> len;
    s.reserve(len);
    s.clear();
    for (uint8_t i = 0; i < len; ++i) {
        S integral;
        data >> integral;
        s.push_back(integral);
    }
    return data;
}

template <typename S, typename std::enable_if<!std::is_integral<S>::value && std::is_base_of<Serializable, S>::value && std::is_default_constructible<S>::value, int>::type = 0>
const SerializationBuffer& operator>>(const SerializationBuffer& data, std::vector<S>& s)
{
    uint8_t len;
    data >> len;
    s.reserve(len);
    s.clear();
    for (uint8_t i = 0; i < len; ++i) {
        s.emplace_back();
        data >> static_cast<Serializable&>(s.back());
    }
    return data;
}

template <typename S, typename std::enable_if<!std::is_integral<S>::value && std::is_base_of<Serializable, S>::value && !std::is_default_constructible<S>::value, int>::type = 0>
const SerializationBuffer& operator>>(const SerializationBuffer& data, std::vector<S>& s)
{
    uint8_t len;
    data >> len;
    s.reserve(len);
    s.clear();
    for (uint8_t i = 0; i < len; ++i) {
        std::shared_ptr<S> object = makeEmpty<S>();
        data >> static_cast<Serializable&>(*object);
        s.push_back(*object);
    }
    return data;
}

}  // namespace csapex

#endif  // CSAPEX_IO_H
