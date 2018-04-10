#ifndef SERIALIZATION_STD_IO_H
#define SERIALIZATION_STD_IO_H

/// COMPONENT
#include <csapex/serialization/serialization_buffer.h>

namespace csapex
{

// STRINGS
SerializationBuffer& operator << (SerializationBuffer& data, const std::string& s);
const SerializationBuffer& operator >> (const SerializationBuffer& data, std::string& s);

// STRING STREAMS
SerializationBuffer& operator << (SerializationBuffer& data, const std::stringstream& s);
const SerializationBuffer& operator >> (const SerializationBuffer& data, std::stringstream& s);

// VECTOR
template <typename S>
SerializationBuffer& operator << (SerializationBuffer& data, const std::vector<S>& s)
{
    apex_assert_lt_hard(s.size(), std::numeric_limits<uint8_t>::max());
    data << (static_cast<uint8_t>(s.size()));
    for(const auto& elem : s) {
        data << elem;
    }
    return data;
}

template <typename S>
const SerializationBuffer& operator >> (const SerializationBuffer& data, std::vector<S>& s)
{
    uint8_t len;
    data >> len;
    s.reserve(len);
    s.clear();
    for(uint8_t i = 0; i < len; ++i) {
        S elem;
        data >> elem;
        s.push_back(elem);
    }
    return data;
}


// PAIR
template <typename S, typename T>
SerializationBuffer& operator << (SerializationBuffer& data, const std::pair<S, T>& s)
{
    data << s.first;
    data << s.second;
    return data;
}

template <typename S, typename T>
const SerializationBuffer& operator >> (const SerializationBuffer& data, std::pair<S, T>& s)
{
    data >> s.first;
    data >> s.second;
    return data;
}

// MAP
template <typename Key, typename Value>
SerializationBuffer& operator << (SerializationBuffer& data, const std::map<Key, Value>& m)
{
    uint64_t size = m.size();
    data << size;
    for(const auto& pair : m) {
        data << pair.first;
        data << pair.second;
    }
    return data;
}

template <typename Key, typename Value>
const SerializationBuffer& operator >> (const SerializationBuffer& data, std::map<Key, Value>& m)
{
    uint64_t size;
    data >> size;
    for(uint64_t i = 0; i < size; ++i) {
        Key key;
        data >> key;
        Value val;
        data >> val;
        m.insert(std::make_pair(key, val));
    }
    return data;
}

// SHARED POINTER (of non-serializable type)
template <typename T,
          typename std::enable_if<!std::is_base_of<Streamable, T>::value,
                                  int>::type = 0>
SerializationBuffer& operator << (SerializationBuffer& data, const std::shared_ptr<T>& s)
{
    if(s) {
        data << true;
        data << *s;
    } else {
        data << false;
    }
    return data;
}

template <typename T,
          typename std::enable_if<!std::is_base_of<Streamable, T>::value &&
                                  std::is_base_of<TokenData, T>::value,
                                  int>::type = 0>
const SerializationBuffer& operator >> (const SerializationBuffer& data, std::shared_ptr<T>& s)
{
    bool valid;
    data >> valid;
    if(!valid) {
        return data;
    }

    // in case T is const, we need to strip that, otherwise we cannot deserialize
    using TT = typename std::remove_const<T>::type;
    std::shared_ptr<TT> res = connection_types::makeEmpty<TT>();
    data >> *res;
    s = res;
    return data;
}

template <typename T,
          typename std::enable_if<!std::is_base_of<Streamable, T>::value &&
                                  !std::is_base_of<TokenData, T>::value &&
                                  std::is_default_constructible<T>::value,
                                  int>::type = 0>
const SerializationBuffer& operator >> (const SerializationBuffer& data, std::shared_ptr<T>& s)
{
    bool valid;
    data >> valid;
    if(!valid) {
        return data;
    }

    // in case T is const, we need to strip that, otherwise we cannot deserialize
    using TT = typename std::remove_const<T>::type;
    std::shared_ptr<TT> res = std::make_shared<TT>();
    data >> *res;
    s = res;
    return data;
}

template <typename T,
          typename std::enable_if<!std::is_base_of<Streamable, T>::value &&
                                  !std::is_base_of<TokenData, T>::value &&
                                  !std::is_default_constructible<T>::value,
                                  int>::type = 0>
const SerializationBuffer& operator >> (const SerializationBuffer& data, std::shared_ptr<T>& s)
{
    bool valid;
    data >> valid;
    if(!valid) {
        return data;
    }

    // in case T is const, we need to strip that, otherwise we cannot deserialize
    using TT = typename std::remove_const<T>::type;
    std::shared_ptr<TT> res(connection_types::makeEmpty<TT>());
    data >> *res;
    s = res;
    return data;
}

}

#endif // SERIALIZATION_STD_IO_H
