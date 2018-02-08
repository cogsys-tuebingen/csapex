#ifndef SERIALIZATION_BUFFER_H
#define SERIALIZATION_BUFFER_H

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/msg/token_traits.h>

/// SYSTEM
#include <vector>
#include <inttypes.h>
#include <string>
#include <sstream>
#include <limits>
#include <typeindex>
#include <boost/any.hpp>

namespace YAML
{
class Node;
}

namespace csapex
{

/**
 * @brief SerializationBuffer
 */
class SerializationBuffer : public std::vector<uint8_t>
{
public:
    static const uint8_t HEADER_LENGTH = 4;
public:
    SerializationBuffer();
    SerializationBuffer(const std::vector<uint8_t>& copy, bool insert_header = false);
    SerializationBuffer(const uint8_t* raw_data, const std::size_t length, bool insert_header = false);

    void finalize();
    void seek(uint32_t p);

    std::string toString() const;

    // SERIALIZABLES
    void write (const Streamable &i);
    void write (const StreamableConstPtr &i);
    StreamablePtr read () const;

    SerializationBuffer& operator << (const Serializable& s);
    const SerializationBuffer& operator >> (Serializable& s) const;

    template <typename T,
              typename std::enable_if<std::is_base_of<Streamable, T>::value,
                                      int>::type = 0>
    SerializationBuffer& operator << (const std::shared_ptr<T>& i)
    {
        write(i);
        return *this;
    }
    template <typename T,
              typename std::enable_if<std::is_base_of<Streamable, T>::value,
                                      int>::type = 0>
    const SerializationBuffer& operator >> (std::shared_ptr<T>& i) const
    {
        i = std::dynamic_pointer_cast<T>(read());
        return *this;
    }

    // INTEGERS
    template <typename T,
              typename std::enable_if<std::is_integral<T>::value,
                                      int>::type = 0>
    SerializationBuffer& operator << (T i)
    {
        std::size_t nbytes = sizeof(T);
        for(std::size_t byte = 0; byte < nbytes; ++byte) {
            uint8_t part = (i >> (byte * 8)) & 0xFF;
            push_back(part);
        }
        return *this;
    }
    template <typename T,
              typename std::enable_if<std::is_integral<T>::value,
                                      int>::type = 0>
    const SerializationBuffer& operator >> (T& i) const
    {
        std::size_t nbytes = sizeof(T);
        T res = 0;
        for(std::size_t byte = 0; byte < nbytes; ++byte) {
            uint8_t part = static_cast<T>(at(pos++));
            res |= (part << (byte * 8));
        }
        i = res;
        return *this;
    }


    // FLOATS
    SerializationBuffer& operator << (float f);
    const SerializationBuffer& operator >> (float& f) const;

    // DOUBLES
    SerializationBuffer& operator << (double d);
    const SerializationBuffer& operator >> (double& d) const;


    // ENUMS
    template <typename T,
              typename std::enable_if<std::is_enum<T>::value,
                                      int>::type = 0>
    SerializationBuffer& operator << (T i)
    {
        push_back(static_cast<uint8_t>(i));
        return *this;
    }
    template <typename T,
              typename std::enable_if<std::is_enum<T>::value,
                                      int>::type = 0>
    const SerializationBuffer& operator >> (T& i) const
    {
        i = static_cast<T>(at(pos++));
        return *this;
    }


    // STRINGS
    SerializationBuffer& operator << (const std::string& s);
    const SerializationBuffer& operator >> (std::string& s) const;


    // STRING STREAMS
    SerializationBuffer& operator << (const std::stringstream& s);
    const SerializationBuffer& operator >> (std::stringstream& s) const;

    // UUID
    SerializationBuffer& operator << (const UUID& s);
    const SerializationBuffer& operator >> (UUID& s) const;

    // Token Data
    SerializationBuffer& operator << (const TokenData& s);
    const SerializationBuffer& operator >> (TokenData& s) const;


    // BOOST ANY
    template <typename T,
              typename std::enable_if<std::is_same<T, boost::any>::value,
                                      int>::type = 0>
    SerializationBuffer& operator << (const T& any)
    {
        return writeAny(any);
    }

    template <typename T,
              typename std::enable_if<std::is_same<T, boost::any>::value,
                                      int>::type = 0>
    const SerializationBuffer& operator >> (T& any) const
    {
        return readAny(any);
    }

    SerializationBuffer& writeAny(const boost::any& any);
    const SerializationBuffer& readAny(boost::any& any) const;

    // VECTOR
    template <typename S>
    SerializationBuffer& operator << (const std::vector<S>& s)
    {
        apex_assert_lt_hard(s.size(), std::numeric_limits<uint8_t>::max());
        operator << (static_cast<uint8_t>(s.size()));
        for(const auto& elem : s) {
            operator << (elem);
        }
        return *this;
    }

    template <typename S>
    const SerializationBuffer& operator >> (std::vector<S>& s) const
    {
        uint8_t len;
        operator >> (len);
        s.reserve(len);
        for(uint8_t i = 0; i < len; ++i) {
            S elem;
            operator >> (elem);
            s.push_back(elem);
        }
        return *this;
    }


    // PAIR
    template <typename S, typename T>
    SerializationBuffer& operator << (const std::pair<S, T>& s)
    {
        operator << (s.first);
        operator << (s.second);
        return *this;
    }

    template <typename S, typename T>
    const SerializationBuffer& operator >> (std::pair<S, T>& s) const
    {
        operator >> (s.first);
        operator >> (s.second);
        return *this;
    }

    // MAP
    template <typename Key, typename Value>
    SerializationBuffer& operator << (const std::map<Key, Value>& m)
    {
        uint64_t size = m.size();
        operator << (size);
        for(const auto& pair : m) {
            operator << (pair.first);
            operator << (pair.second);
        }
        return *this;
    }

    template <typename Key, typename Value>
    const SerializationBuffer& operator >> (std::map<Key, Value>& m) const
    {
        uint64_t size;
        operator >> (size);
        for(uint64_t i = 0; i < size; ++i) {
            Key key;
            operator >> (key);
            Value val;
            operator >> (val);
            m.insert(std::make_pair(key, val));
        }
        return *this;
    }

    // SHARED POINTER (of non-serializable type)
    template <typename T,
              typename std::enable_if<!std::is_base_of<Streamable, T>::value,
                                      int>::type = 0>
    SerializationBuffer& operator << (const std::shared_ptr<T>& s)
    {
        if(s) {
            operator << (true);
            operator << (*s);
        } else {
            operator << (false);
        }
        return *this;
    }

    template <typename T,
              typename std::enable_if<!std::is_base_of<Streamable, T>::value &&
                                      std::is_base_of<TokenData, T>::value,
                                      int>::type = 0>
    const SerializationBuffer& operator >> (std::shared_ptr<T>& s) const
    {
        bool valid;
        operator >>(valid);
        if(!valid) {
            return *this;
        }

        // in case T is const, we need to strip that, otherwise we cannot deserialize
        using TT = typename std::remove_const<T>::type;
        std::shared_ptr<TT> res = connection_types::makeEmpty<TT>();
        operator >> (*res);
        s = res;
        return *this;
    }

    template <typename T,
              typename std::enable_if<!std::is_base_of<Streamable, T>::value &&
                                      !std::is_base_of<TokenData, T>::value &&
                                      std::is_default_constructible<T>::value,
                                      int>::type = 0>
    const SerializationBuffer& operator >> (std::shared_ptr<T>& s) const
    {
        bool valid;
        operator >>(valid);
        if(!valid) {
            return *this;
        }

        // in case T is const, we need to strip that, otherwise we cannot deserialize
        using TT = typename std::remove_const<T>::type;
        std::shared_ptr<TT> res = std::make_shared<TT>();
        operator >> (*res);
        s = res;
        return *this;
    }

    template <typename T,
              typename std::enable_if<!std::is_base_of<Streamable, T>::value &&
                                      !std::is_base_of<TokenData, T>::value &&
                                      !std::is_default_constructible<T>::value,
                                      int>::type = 0>
    const SerializationBuffer& operator >> (std::shared_ptr<T>& s) const
    {
        bool valid;
        operator >>(valid);
        if(!valid) {
            return *this;
        }

        // in case T is const, we need to strip that, otherwise we cannot deserialize
        using TT = typename std::remove_const<T>::type;
        // SerializationBuffer needs to be a friend of T for this to work with private default constructors!
        std::shared_ptr<TT> res(new TT);
        operator >> (*res);
        s = res;
        return *this;
    }


    // YAML
    SerializationBuffer& operator << (const YAML::Node& node);
    const SerializationBuffer& operator >> (YAML::Node& node) const;

private:
    static void init();

private:
    mutable std::size_t pos;

    static bool initialized_;
    static std::map<std::type_index, std::function<void(SerializationBuffer& buffer, const boost::any& a)>> any_serializer;
    static std::map<uint8_t, std::function<void(const SerializationBuffer& buffer, boost::any& a)>> any_deserializer;
};

}

#endif // SERIALIZATION_BUFFER_H
