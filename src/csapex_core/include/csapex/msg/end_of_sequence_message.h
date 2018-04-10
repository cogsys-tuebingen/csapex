#ifndef END_OF_SEQUENCE_MESSAGE_H
#define END_OF_SEQUENCE_MESSAGE_H

/// COMPONENT
#include <csapex/msg/marker_message.h>

namespace csapex
{
namespace connection_types
{

struct CSAPEX_CORE_EXPORT EndOfSequenceMessage : public MarkerMessage
{
public:
    typedef std::shared_ptr<EndOfSequenceMessage> Ptr;

public:
    EndOfSequenceMessage();

protected:
    EndOfSequenceMessage(const std::string& name);

public:
    TokenData::Ptr clone() const override;
    TokenData::Ptr toType() const override;

    std::shared_ptr<Clonable> makeEmptyClone() const override
    {
        return std::shared_ptr<Clonable>(new EndOfSequenceMessage);
    }
    void serialize(SerializationBuffer &data) const override;
    void deserialize(const SerializationBuffer& data) override;
};

template <>
struct type<EndOfSequenceMessage> {
    static std::string name() {
        return "EndOfSequence";
    }
};

}
}

#endif // END_OF_SEQUENCE_MESSAGE_H
