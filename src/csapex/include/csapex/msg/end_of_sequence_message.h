#ifndef END_OF_SEQUENCE_MESSAGE_H
#define END_OF_SEQUENCE_MESSAGE_H

/// COMPONENT
#include <csapex/msg/marker_message.h>

namespace csapex
{
namespace connection_types
{

struct EndOfSequenceMessage : public MarkerMessage
{
public:
    typedef std::shared_ptr<EndOfSequenceMessage> Ptr;

public:
    EndOfSequenceMessage();

public:
    virtual ConnectionType::Ptr clone() const override;
    virtual ConnectionType::Ptr toType() const override;
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
