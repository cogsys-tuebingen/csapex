#ifndef SERIALIZATION_BUFFER_H
#define SERIALIZATION_BUFFER_H

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <vector>
#include <inttypes.h>
#include <string>
#include <sstream>
#include <limits>
#include <boost/any.hpp>

#if G_BYTE_ORDER == G_LITTLE_ENDIAN
union _GFloatIEEE754
{
  float v_float;
  struct {
    uint mantissa : 23;
    uint biased_exponent : 8;
    uint sign : 1;
  } mpn;
};
union _GDoubleIEEE754
{
  double v_double;
  struct {
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
  struct {
    uint sign : 1;
    uint biased_exponent : 8;
    uint mantissa : 23;
  } mpn;
};
union _GDoubleIEEE754
{
  double v_double;
  struct {
    uint sign : 1;
    uint biased_exponent : 11;
    uint mantissa_high : 20;
    uint mantissa_low : 32;
  } mpn;
};
#else /* !G_LITTLE_ENDIAN && !G_BIG_ENDIAN */
#error unknown ENDIAN type
#endif /* !G_LITTLE_ENDIAN && !G_BIG_ENDIAN */

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

    // INTEGERS
    template <typename T,
              typename std::enable_if<std::is_integral<T>::value,
                                      int>::type = 0>
    SerializationBuffer& operator << (T i)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);

        std::size_t nbytes = sizeof(T);
        apex_assert_lte_hard(nbytes, 4);
        for(std::size_t byte = 0; byte < nbytes; ++byte) {
            uint8_t part = (i >> (byte * 8)) & 0xFF;
            push_back(part);
        }
        return *this;
    }
    template <typename T,
              typename std::enable_if<std::is_integral<T>::value,
                                      int>::type = 0>
    SerializationBuffer& operator >> (T& i)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);

        std::size_t nbytes = sizeof(T);
        apex_assert_lte_hard(nbytes, 4);
        T res = 0;
        for(std::size_t byte = 0; byte < nbytes; ++byte) {
            uint8_t part = static_cast<T>(at(pos++));
            res |= (part << (byte * 8));
        }
        i = res;
        return *this;
    }


    // FLOATS
    SerializationBuffer& operator << (float f)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);
        _GFloatIEEE754 ieee;
        ieee.v_float = f;

        /***
         * sign  | mantissa    |  exponent
         *   1         23             8
         *   1   | 2 - 24      | 25 - 32
         */
        uint8_t bytes[4];
        //memset(&bytes[0], 0, 4);
        bytes[0] = ieee.mpn.sign << 7;
        bytes[0] |= (ieee.mpn.mantissa >> (23 - 7));
        bytes[1] = (ieee.mpn.mantissa >> (23 - 7 - 8)) & 0xFF;
        bytes[2] = (ieee.mpn.mantissa >> (23 - 7 - 16)) & 0xFF;
        bytes[3] = ieee.mpn.biased_exponent;

        for(uint8_t i = 0; i < 4; ++i) {
            push_back(bytes[i]);
        }

        return *this;
    }
    SerializationBuffer& operator >> (float& f)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);

        uint8_t bytes[4];
        for(uint8_t i = 0; i < 4; ++i) {
            bytes[i] = at(pos++);
        }

        _GFloatIEEE754 ieee;

        ieee.mpn.sign = (bytes[0] & (1 << 7)) ? 1 : 0;
        ieee.mpn.mantissa = ((bytes[0] & (~(1 << 7))) << (23 - 7)) |
                (bytes[1] << (23 - 7 - 8)) |
                (bytes[2] << (23 - 7 - 16));
        ieee.mpn.biased_exponent = bytes[3];

        f = ieee.v_float;

        return *this;
    }


    // DOUBLES
    SerializationBuffer& operator << (double d)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);
        _GDoubleIEEE754 ieee;
        ieee.v_double = d;

        /***
         * sign  | exponent   |  mantissa_high   | mantissa_low
         *   1         11             20         |   32
         *   1   | 2 - 12         | 13 - 32      |   32
         */
        uint8_t bytes[8];
        //memset(&bytes[0], 0, 4);
        bytes[0] = ieee.mpn.sign << 7;
        bytes[0] |= (ieee.mpn.biased_exponent >> (11 - 7));
        bytes[1] = (ieee.mpn.biased_exponent << 4);
        bytes[1] |= (ieee.mpn.mantissa_high >> (20 - 4)) & 0xFF;
        bytes[2] = (ieee.mpn.mantissa_high >> (20 - 4 - 8)) & 0xFF;
        bytes[3] = (ieee.mpn.mantissa_high >> (20 - 4 - 16)) & 0xFF;
        bytes[4] = (ieee.mpn.mantissa_low >> (3*8)) & 0xFF;
        bytes[5] = (ieee.mpn.mantissa_low >> (2*8)) & 0xFF;
        bytes[6] = (ieee.mpn.mantissa_low >> (1*8)) & 0xFF;
        bytes[7] = (ieee.mpn.mantissa_low >> (0*8)) & 0xFF;


        for(uint8_t i = 0; i < 8; ++i) {
            push_back(bytes[i]);
        }

        return *this;
    }
    SerializationBuffer& operator >> (double& d)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);

        uint8_t bytes[8];
        for(uint8_t i = 0; i < 8; ++i) {
            bytes[i] = at(pos++);
        }

        _GDoubleIEEE754 ieee;

        ieee.mpn.sign = (bytes[0] & (1 << 7)) ? 1 : 0;
        ieee.mpn.biased_exponent = ((bytes[0] & (~(1 << 7))) << (11 - 7)) |
                ((bytes[1] & (0xF0)) >> 4);
        ieee.mpn.mantissa_high = ((bytes[1] & (~(1 << 7))) << (20 - 4)) |
                (bytes[2] << (20 - 4 - 8)) |
                (bytes[3] << (20 - 4 - 16));

        ieee.mpn.mantissa_low = (bytes[4] << (3*8)) |
                (bytes[5] << (2*8)) |
                (bytes[6] << (1*8)) |
                (bytes[7] << (0*8));

        d = ieee.v_double;

        return *this;
    }


    // ENUMS
    template <typename T,
              typename std::enable_if<std::is_enum<T>::value,
                                      int>::type = 0>
    SerializationBuffer& operator << (T i)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);
        push_back(static_cast<uint8_t>(i));
        return *this;
    }
    template <typename T,
              typename std::enable_if<std::is_enum<T>::value,
                                      int>::type = 0>
    SerializationBuffer& operator >> (T& i)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);
        i = static_cast<T>(at(pos++));
        return *this;
    }


    // STRINGS
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


    // STRING STREAMS
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

    // UUID
    SerializationBuffer& operator << (const UUID& s)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);
        operator << (s.getFullName());
        return *this;
    }

    SerializationBuffer& operator >> (UUID& s)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);
        std::string full_name;
        operator >> (full_name);
        s = UUIDProvider::makeUUID_without_parent(full_name);
        return *this;
    }


    // BOOST ANY
    SerializationBuffer& operator << (const boost::any& any)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);

        if(any.type() == typeid(int)) {
            operator << ((uint8_t) 1);
            operator << (boost::any_cast<int> (any));

        } else if(any.type() == typeid(double)) {
            operator << ((uint8_t) 2);
            operator << (boost::any_cast<double> (any));

        } else if(any.type() == typeid(bool)) {
            operator << ((uint8_t) 3);
            operator << (boost::any_cast<bool> (any));

        } else if(any.type() == typeid(std::string)) {
            operator << ((uint8_t) 4);
            operator << (boost::any_cast<std::string> (any));

        } else if(any.type() == typeid(long)) {
            operator << ((uint8_t) 5);
            operator << (boost::any_cast<long> (any));

        } else {
            operator << ((uint8_t) 0);
        }

        return *this;
    }

    SerializationBuffer& operator >> (boost::any& any)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);

        uint8_t type;
        operator >> (type);

        switch(type) {
        case 1:
        {
            int v;
            operator >> (v);
            any = v;
        }
            break;
        case 2:
        {
            double v;
            operator >> (v);
            any = v;
        }
            break;
        case 3:
        {
            bool v;
            operator >> (v);
            any = v;
        }
            break;
        case 4:
        {
            std::string v;
            operator >> (v);
            any = v;
        }
            break;
        case 5:
        {
            long v;
            operator >> (v);
            any = v;
        }
            break;
        }
        return *this;
    }

    /// VECTOR
    template <typename S>
    SerializationBuffer& operator << (const std::vector<S>& s)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);
        operator << (s.size());
        for(const auto& elem : s) {
            operator << (elem);
        }
        return *this;
    }

    template <typename S>
    SerializationBuffer& operator >> (std::vector<S>& s)
    {
        apex_assert_gte_hard(pos, HEADER_LENGTH);
        uint8_t len;
        operator >> (len);
        s.reserve(len);
        for(uint8_t i = 0; i < len; ++i) {
            S elem;
            operator >> (elem);
            s.emplace_back(elem);
        }
        return *this;
    }

private:
    std::size_t pos;
};

}

#endif // SERIALIZATION_BUFFER_H
