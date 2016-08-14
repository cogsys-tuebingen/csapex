#ifndef NO_MESSAGE_H
#define NO_MESSAGE_H

/// COMPONENT
#include <csapex/msg/marker_message.h>

namespace csapex
{
namespace connection_types
{

struct CSAPEX_EXPORT NoMessage : public MarkerMessage
{
public:
    typedef std::shared_ptr<NoMessage> Ptr;

public:
    NoMessage();

public:
    virtual TokenData::Ptr clone() const override;
    virtual TokenData::Ptr toType() const override;
};

template <>
struct type<NoMessage> {
    static std::string name() {
        return "Nothing";
    }
};

}
}

#endif // NO_MESSAGE_H

