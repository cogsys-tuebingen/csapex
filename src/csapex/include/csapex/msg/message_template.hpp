#ifndef MESSAGE_TEMPLATE_H
#define MESSAGE_TEMPLATE_H

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/token_traits.h>
#include <csapex/msg/io.h>

/// SYSTEM
#include <type_traits>

namespace csapex
{
namespace connection_types
{

template <typename Type, bool integral>
struct MessageTemplateContainer : public Type
{
    using value_type = Type;

    Type& getValue() {
        return *this;
    }

    static bool acceptsConnectionFrom(const TokenData* other_side)
    {
        return dynamic_cast<const Type*>(other_side);
    }
};

template <typename Type>
struct MessageTemplateContainer<Type, true>
{
    using value_type = Type;

    Type& getValue() {
        return value_impl;
    }

    static bool acceptsConnectionFrom(const TokenData* other_side)
    {
        return dynamic_cast<const MessageTemplateContainer<Type, true>*>(other_side);
    }

private:
    Type value_impl;
};

struct MessageTemplateBase
{};

template <typename Type, class Instance>
class MessageTemplate : public Message, public MessageTemplateContainer<Type, std::is_integral<Type>::value>, public MessageTemplateBase
{
public:
    typedef std::shared_ptr<Instance> Ptr;
    typedef std::shared_ptr<const Instance> ConstPtr;

    typedef MessageTemplateContainer<Type, std::is_integral<Type>::value> Parent;
    typedef MessageTemplate<Type,Instance> Self;

    MessageTemplate( const std::string& frame_id = "/", Message::Stamp stamp = 0)
        : Message(type<Instance>::name(), frame_id, stamp),
          value(Parent::getValue())
    {}


    Self& operator = (const Self& other)
    {
        Parent::operator=(other);
        value = Parent::getValue();
        return *this;
    }

    virtual TokenData::Ptr clone() const override
    {
        return cloneInstance();
    }

    virtual TokenData::Ptr toType() const override
    {
        Ptr new_msg(new Instance);
        return new_msg;
    }


    bool acceptsConnectionFrom(const TokenData* other_side) const override
    {
        return Parent::acceptsConnectionFrom(other_side);
    }

    Type& value;

protected:
    Ptr cloneInstance() const
    {
        Ptr new_msg(new Instance);
        new_msg->frame_id = frame_id;
        new_msg->stamp_micro_seconds = stamp_micro_seconds;
        new_msg->value = value;
        return new_msg;
    }

};


}



/// CASTING
///

namespace msg
{

template <typename Instance, typename S>
struct MessageCaster<Instance, S, typename std::enable_if<std::is_base_of<connection_types::MessageTemplateBase, Instance>::value>::type>
{
    using V = typename Instance::value_type;
    static std::shared_ptr<Instance const> constcast(const std::shared_ptr<S const>& msg)
    {
        // if we can dynamic cast directly, use that
        if(auto direct = std::dynamic_pointer_cast<Instance const>(msg)) {
            return direct;
        }

        // if we can cast the message to the value type, create a new message of that type
        if(auto* casted = dynamic_cast<V const*>(msg.get())) {
            auto res = connection_types::makeEmptyMessage<Instance>();

            // copy specific data
            res->value = *casted;

            if(auto other_base = dynamic_cast<connection_types::Message const*>(msg.get())) {
                // copy generic data
                dynamic_cast<connection_types::Message&>(*res) = *other_base;
            }
            return res;
        }

        // otherwise we cannot do anything
        return nullptr;
    }
    static std::shared_ptr<Instance> cast(const std::shared_ptr<S>& msg)
    {
        // if we can dynamic cast directly, use that
        if(auto direct = std::dynamic_pointer_cast<Instance>(msg)) {
            return direct;
        }

        // if we can cast the message to the value type, create a new message of that type
        if(auto* casted = dynamic_cast<V*>(msg.get())) {
            auto res = connection_types::makeEmptyMessage<Instance>();

            // copy specific data
            res->value = *casted;

            if(auto other_base = dynamic_cast<connection_types::Message*>(msg.get())) {
                // copy generic data
                dynamic_cast<connection_types::Message&>(*res) = *other_base;
            }
            return res;
        }

        // otherwise we cannot do anything
        return nullptr;
    }
};

}
}

#endif // MESSAGE_TEMPLATE_H
