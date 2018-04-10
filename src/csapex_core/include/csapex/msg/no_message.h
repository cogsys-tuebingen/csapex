#ifndef NO_MESSAGE_H
#define NO_MESSAGE_H

/// COMPONENT
#include <csapex/msg/marker_message.h>

namespace csapex
{
namespace connection_types
{

struct CSAPEX_CORE_EXPORT NoMessage : public MarkerMessage
{
public:
    typedef std::shared_ptr<NoMessage> Ptr;

public:
    NoMessage();

public:
    virtual TokenData::Ptr clone() const override;
    virtual TokenData::Ptr toType() const override;

    std::shared_ptr<Clonable> makeEmptyClone() const override
    {
        return std::shared_ptr<Clonable>(new NoMessage);
    }
    void serialize(SerializationBuffer &data) const override;
    void deserialize(const SerializationBuffer& data) override;
};

template <>
struct type<NoMessage> {
    static std::string name() {
        return "Nothing";
    }
};

template <>
inline std::shared_ptr<NoMessage> makeEmpty<NoMessage>()
{
    static std::shared_ptr<NoMessage> instance(new NoMessage);
    return instance;
}


}
}

#endif // NO_MESSAGE_H

