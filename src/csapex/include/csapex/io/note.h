#ifndef NOTE_H
#define NOTE_H

/// PROJECT
#include <csapex/serialization/streamable.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <string>

namespace csapex
{

namespace io
{

class Note : public Streamable
{
public:
    Note(AUUID uuid);

    static const uint8_t PACKET_TYPE_ID = 7;

    virtual uint8_t getPacketType() const override;
    virtual std::string getType() const = 0;

    AUUID getAUUID() const;

    virtual void serialize(SerializationBuffer &data) const override;
    virtual void deserialize(const SerializationBuffer& data) override;

protected:
    AUUID uuid_;
};

}
}

#endif // NOTE_H
