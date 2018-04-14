#ifndef MESSAGE_TEMPLATE_H
#define MESSAGE_TEMPLATE_H

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/token_traits.h>
#include <csapex/msg/io.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/serialization/io/csapex_io.h>

/// SYSTEM
#include <type_traits>

namespace csapex
{
namespace connection_types
{


/**
 * @brief The MessageTemplateContainer struct wraps non integral types.
 *        The class derives from the data type so that it can be dynamically casted to and from it.
 *        A reference 'value' is kept up to date, which will always refer to the value that is derived from.
 * @tparam Type the type to wrap
 * @tparam integral true, iff type is integral
 */
template <typename Type, bool integral>
struct MessageTemplateContainer : public Type
{
    using value_type = Type;

    MessageTemplateContainer()
        : value(*this)
    {}
    MessageTemplateContainer(const MessageTemplateContainer& other)
        : Type(other), value(*this)
    {
        *this = other.value;
    }
    MessageTemplateContainer(const Type& v)
        : value(*this)
    {
        value = v;
    }
    MessageTemplateContainer(Type&& v)
        : value(*this)
    {
        value = std::move(v);
    }

    MessageTemplateContainer& operator = (const MessageTemplateContainer & copy)
    {
        value = copy.value;
        return *this;
    }
    MessageTemplateContainer& operator = (MessageTemplateContainer&& copy)
    {
        value = std::move(copy.value);
        return *this;
    }

    MessageTemplateContainer& operator = (const Type & other_value)
    {
        value = other_value;
        return *this;
    }
    MessageTemplateContainer& operator = (Type && other_value)
    {
        value = std::move(other_value);
        return *this;
    }

    static bool acceptsConnectionFrom(const TokenData* other_side)
    {
        return dynamic_cast<const Type*>(other_side);
    }

public:
    Type& value;
};

/**
 * @brief The MessageTemplateContainer struct wraps non integral types.
 *        This is a specialization for integral types, which we cannot derive from.
 * @tparam Type the type to wrap
 * @tparam integral true, iff type is integral
 */
template <typename Type>
struct MessageTemplateContainer<Type, true>
{
    using value_type = Type;

    static bool acceptsConnectionFrom(const TokenData* other_side)
    {
        return dynamic_cast<const MessageTemplateContainer<Type, true>*>(other_side);
    }

public:
    Type value;
};

/**
 * @brief The MessageTemplateBase struct is a marker for RTTI
 */
struct MessageTemplateBase
{};

/**
 * @brief The MessageTemplate class generates a message from any datatype
 * @tparam Type The type to wrap in this message
 * @tparam Instance The instance that derives from this class
 *         This uses the "curiously recurring template pattern", so child classes must be of the form
 *         class Instance : public MessagesTemplate<Type, Instance>
 */
template <typename Type, class Instance>
class MessageTemplate : public Message, public MessageTemplateContainer<Type, std::is_integral<Type>::value>, public MessageTemplateBase
{
public:
    typedef std::shared_ptr<Instance> Ptr;
    typedef std::shared_ptr<const Instance> ConstPtr;

    typedef MessageTemplateContainer<Type, std::is_integral<Type>::value> ValueContainer;
    typedef MessageTemplate<Type,Instance> Self;

    explicit MessageTemplate( const std::string& frame_id = "/", Message::Stamp stamp = 0)
        : Message(type<Instance>::name(), frame_id, stamp)
    {}

    MessageTemplate( const Self& copy )
        : Message(type<Instance>::name(), copy.frame_id, copy.stamp_micro_seconds),
          ValueContainer(static_cast<const ValueContainer&>(copy))
    {}

    MessageTemplate( Self&& moved )
        : Message(type<Instance>::name(), moved.frame_id, moved.stamp_micro_seconds),
          ValueContainer(static_cast<ValueContainer&&>(moved))
    {}


    Self& operator = (const Self& other)
    {
        ValueContainer::operator=(other);
        return *this;
    }
    Self& operator = (Self&& other)
    {
        ValueContainer::operator=(std::move(other));
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
        return ValueContainer::acceptsConnectionFrom(other_side);
    }    

    std::shared_ptr<Clonable> makeEmptyClone() const override
    {
        return std::shared_ptr<Clonable>(new Self);
    }
    void serialize(SerializationBuffer &data) const override
    {
        data << ValueContainer::value;
    }
    void deserialize(const SerializationBuffer& data) override
    {
        data >> ValueContainer::value;
    }

protected:
    Ptr cloneInstance() const
    {
        Ptr new_msg(new Instance);
        new_msg->frame_id = frame_id;
        new_msg->stamp_micro_seconds = stamp_micro_seconds;
        new_msg->value = ValueContainer::value;
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
