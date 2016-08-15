#ifndef END_OF_SEQUENCE_MESSAGE_H
#define END_OF_SEQUENCE_MESSAGE_H

/// COMPONENT
#include <csapex/msg/marker_message.h>

namespace csapex
{
namespace connection_types
{

struct CSAPEX_EXPORT EndOfSequenceMessage : public MarkerMessage
{
public:
    typedef std::shared_ptr<EndOfSequenceMessage> Ptr;

public:
    EndOfSequenceMessage();

protected:
    EndOfSequenceMessage(const std::string& name);

public:
    virtual TokenData::Ptr clone() const override;
    virtual TokenData::Ptr toType() const override;
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
