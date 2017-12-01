#ifndef RAW_MESSAGE_H
#define RAW_MESSAGE_H

/// PROJECT
#include <csapex/serialization/streamable.h>
#include <csapex/core/csapex_core.h>
#include <csapex/io/io_fwd.h>

/// SYSTEM
#include <string>

namespace csapex
{

class RawMessage : public Streamable
{
public:
    RawMessage();
    RawMessage(const std::vector<uint8_t>& data, const AUUID& target);
    RawMessage(const char *data, std::size_t len, const AUUID& target);

    static const uint8_t PACKET_TYPE_ID = 10;

    virtual uint8_t getPacketType() const override;

    std::vector<uint8_t> getData() const;
    AUUID getUUID() const;

    virtual void serialize(SerializationBuffer &data) const;
    virtual void deserialize(const SerializationBuffer& data);

private:
    virtual std::shared_ptr<Clonable> makeEmptyClone() const;

private:
    mutable std::vector<uint8_t> data_;
    AUUID uuid_;
};

}

#endif // RAW_MESSAGE_H
