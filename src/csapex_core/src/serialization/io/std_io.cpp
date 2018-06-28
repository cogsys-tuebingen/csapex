/// HEADER
#include <csapex/serialization/io/std_io.h>

using namespace csapex;

// STRINGS
SerializationBuffer& csapex::operator << (SerializationBuffer& data, const std::string& s)
{
    apex_assert_lt_hard(s.size(), std::numeric_limits<uint16_t>::max());
    data << static_cast<uint16_t>(s.size());
    data.writeRaw(s.data(), s.size());
    return data;
}

const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, std::string& s)
{
    uint16_t str_len;
    data >> str_len;
    if(str_len > 0) {
        s.resize(str_len);
        data.readRaw(&s.at(0), str_len);
    }
    return data;
}


// STRING STREAMS
SerializationBuffer& csapex::operator << (SerializationBuffer& data, const std::stringstream& s)
{
    data << s.str();
    return data;
}

const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, std::stringstream& s)
{
    std::string str;
    data >> str;
    s.str(str);
    return data;
}
