#ifndef SERIALIZATION_BUFFER_H
#define SERIALIZATION_BUFFER_H

/// PROJECT
#include <csapex/utility/assert.h>

/// SYSTEM
#include <vector>
#include <inttypes.h>
#include <string>
#include <sstream>
#include <limits>

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
    SerializationBuffer()
        : pos(HEADER_LENGTH)
    {
        // the header is always 4 byte
        insert(end(), HEADER_LENGTH, 0);
    }

    void finalize()
    {
        at(0) = size();
    }

    std::string toString() const
    {
        std::stringstream res;
        for(std::size_t i = 0; i < size(); ++i) {
            if((i%8) == 0) {
                res << std::hex << i << ":\t\t" << std::dec;
            }

            res << (int) at(i);


            if((i%8) != 7) {
                res << '\t';
            } else {
                res << '\n';
            }
        }
        return res.str();
    }

    template <typename T,
              typename std::enable_if<std::is_integral<T>::value ||
                                      std::is_enum<T>::value,
                                      int>::type = 0>
    SerializationBuffer& operator << (T i)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);
        push_back(static_cast<uint8_t>(i));
        return *this;
    }
    template <typename T,
              typename std::enable_if<std::is_integral<T>::value ||
                                      std::is_enum<T>::value,
                                      int>::type = 0>
    SerializationBuffer& operator >> (T& i)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);
        i = static_cast<T>(at(pos++));
        return *this;
    }


    SerializationBuffer& operator << (const std::string& s)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);
        apex_assert_lt_hard(s.size(), std::numeric_limits<uint8_t>::max());
        push_back(s.size());
        for(const char& ch : s) {
            push_back(ch);
        }
        return *this;
    }

    SerializationBuffer& operator >> (std::string& s)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);
        uint8_t str_len = at(pos++);
        apex_assert_lte_hard(pos + str_len, size());

        s.resize(str_len, ' ');
        for(std::size_t i = 0; i < str_len; ++i) {
            s[i] = at(pos++);
        }
        return *this;
    }


    SerializationBuffer& operator << (const std::stringstream& s)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);
        operator << (s.str());
        return *this;
    }

    SerializationBuffer& operator >> (std::stringstream& s)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);
        std::string str;
        operator >> (str);
        s.str(str);
        return *this;
    }

    std::size_t pos;
};

}

#endif // SERIALIZATION_BUFFER_H
