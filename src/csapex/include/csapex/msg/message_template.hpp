#ifndef MESSAGE_TEMPLATE_H
#define MESSAGE_TEMPLATE_H

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/message_traits.h>

namespace csapex {
namespace connection_types {

template <typename Type, class Instance>
struct MessageTemplate : public Message {
    typedef boost::shared_ptr<Instance> Ptr;

    MessageTemplate( const std::string& frame_id = "/", Message::Stamp stamp = 0)
        : Message(type<Instance>::name(), frame_id, stamp)
    {}

    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new Instance);
        new_msg->frame_id = frame_id;
        new_msg->stamp = stamp;
        new_msg->value = value;
        return new_msg;
    }

    virtual ConnectionType::Ptr toType() {
        Ptr new_msg(new Instance);
        return new_msg;
    }


    bool acceptsConnectionFrom(const ConnectionType* other_side) const {
        return rawName() == other_side->rawName();
    }

    Type value;
};


}
}

#endif // MESSAGE_TEMPLATE_H
