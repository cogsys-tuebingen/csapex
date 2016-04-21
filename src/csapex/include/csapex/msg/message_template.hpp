#ifndef MESSAGE_TEMPLATE_H
#define MESSAGE_TEMPLATE_H

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/message_traits.h>

namespace csapex
{
namespace connection_types
{

template <typename Type, class Instance>
class MessageTemplate : public Message
{
public:
    typedef std::shared_ptr<Instance> Ptr;
    typedef std::shared_ptr<const Instance> ConstPtr;

    MessageTemplate( const std::string& frame_id = "/", Message::Stamp stamp = 0)
        : Message(type<Instance>::name(), frame_id, stamp)
    {}

    virtual Token::Ptr clone() const override
    {
        Ptr new_msg(new Instance);
        new_msg->frame_id = frame_id;
        new_msg->stamp_micro_seconds = stamp_micro_seconds;
        new_msg->value = value;
        return new_msg;
    }

    virtual Token::Ptr toType() const override
    {
        Ptr new_msg(new Instance);
        return new_msg;
    }


    bool acceptsConnectionFrom(const Token* other_side) const override
    {
        return typeName() == other_side->typeName();
    }

    Type value;
};


}
}

#endif // MESSAGE_TEMPLATE_H
