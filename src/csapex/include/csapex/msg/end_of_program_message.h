#ifndef END_OF_PROGRAM_MESSAGE_H
#define END_OF_PROGRAM_MESSAGE_H

/// COMPONENT
#include <csapex/msg/end_of_sequence_message.h>

namespace csapex
{
namespace connection_types
{

struct CSAPEX_EXPORT EndOfProgramMessage : public EndOfSequenceMessage
{
public:
    typedef std::shared_ptr<EndOfProgramMessage> Ptr;

public:
    EndOfProgramMessage();

public:
    virtual TokenData::Ptr clone() const override;
    virtual TokenData::Ptr toType() const override;
};

template <>
struct type<EndOfProgramMessage> {
    static std::string name() {
        return "EndOfProgram";
    }
};

}
}

#endif // END_OF_PROGRAM_MESSAGE_H
