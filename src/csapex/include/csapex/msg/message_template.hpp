#ifndef MESSAGE_TEMPLATE_H
#define MESSAGE_TEMPLATE_H

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/message_traits.h>

namespace csapex {
namespace connection_types {

template <typename Type, class Instance>
struct MessageTemplate : public Message {
    typedef std::shared_ptr<Instance> Ptr;
    typedef std::shared_ptr<const Instance> ConstPtr;

    MessageTemplate( const std::string& frame_id = "/", Message::Stamp stamp = 0)
        : Message(type<Instance>::name(), frame_id, stamp)
    {}

    virtual ConnectionType::Ptr clone() const override
    {
        Ptr new_msg(new Instance);
        new_msg->frame_id = frame_id;
        new_msg->stamp = stamp;
        new_msg->value = value;
        return new_msg;
    }

    virtual ConnectionType::Ptr toType() const override
    {
        Ptr new_msg(new Instance);
        return new_msg;
    }


    bool acceptsConnectionFrom(const ConnectionType* other_side) const override
    {
        return rawName() == other_side->rawName();
    }

    Type value;
};


}
}

#endif // MESSAGE_TEMPLATE_H
