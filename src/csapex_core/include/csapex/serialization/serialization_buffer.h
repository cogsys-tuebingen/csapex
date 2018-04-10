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

    void seek(uint32_t p) const;
    void rewind() const;
    void advance(uint32_t distance) const;

    uint32_t getPos() const;

    std::string toString() const;

    // SERIALIZABLES
    void write (const Streamable &i);
    void write (const StreamableConstPtr &i);
    StreamablePtr read () const;

    void writeRaw(const char* data, const std::size_t length);
    void writeRaw(const uint8_t* data, const std::size_t length);
    void readRaw(char* data, const std::size_t length) const;
    void readRaw(uint8_t* data, const std::size_t length) const;


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
