/// HEADER
#include <csapex/io/raw_message.h>

/// PROJECT
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

uint8_t RawMessage::getPacketType() const
{
    return PACKET_TYPE_ID;
}

RawMessage::RawMessage()
{

}

RawMessage::RawMessage(std::vector<uint8_t> data, const AUUID& target)
    : data_(data), uuid_(target)
{

}
RawMessage::RawMessage(const char* data, std::size_t len, const AUUID& target)
    : data_(len), uuid_(target)
{
    uint8_t* dst = data_.data();
    const char* src = data;
    for(std::size_t i = 0; i < len; ++i, ++src, ++dst) {
        *dst = *src;
    }
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


std::shared_ptr<Clonable> RawMessage::makeEmptyClone() const
{
    return std::make_shared<RawMessage>();
}
