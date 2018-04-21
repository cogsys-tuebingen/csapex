/// HEADER
#include <csapex/io/raw_message.h>

/// PROJECT
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>

/// SYSTEM
#include <iostream>
#include <cstring>

using namespace csapex;

uint8_t RawMessage::getPacketType() const
{
    return PACKET_TYPE_ID;
}

RawMessage::RawMessage()
{

}

RawMessage::RawMessage(const std::vector<uint8_t>& data, const AUUID& target)
    : data_(data), uuid_(target)
{

}

RawMessage::RawMessage(const char* data, std::size_t len, const AUUID& target)
    : data_(len), uuid_(target)
{
    std::memcpy(data_.data(), data, len);
}

std::vector<uint8_t> RawMessage::getData() const
{
    return data_;
}

AUUID RawMessage::getUUID() const
{
    return uuid_;
}

void RawMessage::serialize(SerializationBuffer &data) const
{
    data << uuid_;
    std::size_t n = data_.size();
    data << n;

    uint8_t* src = data_.data();
    for(std::size_t i = 0; i < n; ++i, ++src) {
        data << *src;
    }
}
void RawMessage::deserialize(const SerializationBuffer& data)
{
    data >> uuid_;
    std::size_t n;
    data >> n;

    data_.resize(n, 0);
    uint8_t* dst = data_.data();
    for(std::size_t i = 0; i < n; ++i, ++dst) {
        data >> *dst;
    }
}
