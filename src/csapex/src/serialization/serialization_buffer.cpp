/// HEADER
#include <csapex/serialization/serialization_buffer.h>

/// PROJECT
#include <csapex/serialization/serializable.h>
#include <csapex/serialization/packet_serializer.h>

using namespace csapex;

void SerializationBuffer::write(const SerializableConstPtr &i)
{
    bool null = (i == nullptr);
    operator << (null);
    if(!null) {
        PacketSerializer::instance().serialize(i, *this);
    }
}

SerializablePtr SerializationBuffer::read()
{
    bool null;
    operator >> (null);
    if(!null) {
        return PacketSerializer::instance().deserialize(*this);
    }
    return nullptr;
}

SerializationBuffer& SerializationBuffer::writeAny (const boost::any& any)
{
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

SerializationBuffer& SerializationBuffer::readAny (boost::any& any)
{
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

SerializationBuffer& SerializationBuffer::operator << (float f)
{
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
SerializationBuffer& SerializationBuffer::operator >> (float& f)
{
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
SerializationBuffer& SerializationBuffer::operator << (double d)
{
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
SerializationBuffer& SerializationBuffer::operator >> (double& d)
{
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


// STRINGS
SerializationBuffer& SerializationBuffer::operator << (const std::string& s)
{
    apex_assert_lt_hard(s.size(), std::numeric_limits<uint16_t>::max());
    operator << (static_cast<uint16_t>(s.size()));
    for(const char& ch : s) {
        push_back(ch);
    }
    return *this;
}

SerializationBuffer& SerializationBuffer::operator >> (std::string& s)
{
    uint16_t str_len;
    operator >> (str_len);
    apex_assert_lte_hard(pos + str_len, size());

    s.resize(str_len, ' ');
    for(std::size_t i = 0; i < str_len; ++i) {
        s[i] = at(pos++);
    }
    return *this;
}


// STRING STREAMS
SerializationBuffer& SerializationBuffer::operator << (const std::stringstream& s)
{
    operator << (s.str());
    return *this;
}

SerializationBuffer& SerializationBuffer::operator >> (std::stringstream& s)
{
    std::string str;
    operator >> (str);
    s.str(str);
    return *this;
}

// UUID
SerializationBuffer& SerializationBuffer::operator << (const UUID& s)
{
    operator << (s.getFullName());
    return *this;
}

SerializationBuffer& SerializationBuffer::operator >> (UUID& s)
{
    std::string full_name;
    operator >> (full_name);
    s = UUIDProvider::makeUUID_without_parent(full_name);
    return *this;
}
