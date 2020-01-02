#ifndef RAW_MESSAGE_H
#define RAW_MESSAGE_H

/// PROJECT
#include <csapex/serialization/streamable.h>
#include <csapex/core/csapex_core.h>
#include <csapex/io/remote_io_fwd.h>

/// SYSTEM
#include <string>

namespace csapex
{
class RawMessage : public Streamable
{
protected:
    CLONABLE_IMPLEMENTATION(RawMessage);

public:
    RawMessage();
    RawMessage(const std::vector<uint8_t>& data, const AUUID& target);
    RawMessage(const char* data, std::size_t len, const AUUID& target);

    static const uint8_t PACKET_TYPE_ID = 10;

    uint8_t getPacketType() const override;

    std::vector<uint8_t> getData() const;
    AUUID getUUID() const;

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

private:
    mutable std::vector<uint8_t> data_;
    AUUID uuid_;
};

}  // namespace csapex

#endif  // RAW_MESSAGE_H
