#ifndef GENERIC_VALUE_MESSAGE_H
#define GENERIC_VALUE_MESSAGE_H

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/utility/register_msg.h>
#include <csapex/serialization/message_serializer.h>
#include <csapex/msg/io.h>
#include <csapex/utility/string.hpp>

// TODO remove
#include <iostream>

namespace csapex {
namespace connection_types {


struct ValueMessageBase
{
    virtual bool isArithmetic() const = 0;

    virtual int64_t asInteger() const = 0;
    virtual double asDouble() const = 0;
    virtual std::string asString() const = 0;
};

template <typename Type>
struct GenericValueMessage : public Message, public ValueMessageBase
{
    typedef std::shared_ptr<GenericValueMessage<Type> > Ptr;
    typedef std::shared_ptr<GenericValueMessage<Type> const> ConstPtr;

    explicit GenericValueMessage(const Type& value = Type(), const std::string& frame_id = "/", Message::Stamp stamp = 0)
        : Message(type< GenericValueMessage<Type> >::name(), frame_id, stamp),
          value(value)
    {
        static csapex::DirectMessageConstructorRegistered<connection_types::GenericValueMessage, Type> reg_c;
        static csapex::DirectMessageSerializerRegistered<connection_types::GenericValueMessage, Type> reg_s;
    }

    virtual TokenData::Ptr clone() const override
    {
        return std::make_shared<GenericValueMessage<Type>>(value, frame_id, stamp_micro_seconds);
    }

    virtual TokenData::Ptr toType() const override
    {
        return std::make_shared<GenericValueMessage<Type>>();
    }

    bool acceptsConnectionFrom(const TokenData* other_side) const override
    {
        if(dynamic_cast<const GenericValueMessage<Type>*>(other_side)){
            return true;
        }

        if(const ValueMessageBase* other_value =
                dynamic_cast<const ValueMessageBase*>(other_side)) {
            if(isArithmetic() && other_value->isArithmetic()) {
                return true;

            } else if(std::is_base_of<std::string, Type>::value) {
                // all messages can be converted to string
                return true;
            }
        }
        return descriptiveName() == other_side->descriptiveName();
    }

    Type getValue()
    {
        return value;
    }

    bool isArithmetic() const override
    {
        return std::is_arithmetic<Type>::value;
    }

    int64_t asInteger() const override
    {
        return asIntegerImpl<Type>();
    }
    double asDouble() const override
    {
        return asDoubleImpl<Type>();
    }
    std::string asString() const override
    {
        return asStringImpl<Type>();
    }

    template <typename T>
    int64_t asIntegerImpl(typename std::enable_if<std::is_arithmetic<T>::value>::type* = 0) const
    {
        return value;
    }

    template <typename T>
    int64_t asIntegerImpl(typename std::enable_if<!std::is_arithmetic<T>::value>::type* = 0) const
    {
        throw std::runtime_error("type not convertible to int.");
    }

    template <typename T>
    double asDoubleImpl(typename std::enable_if<std::is_arithmetic<T>::value>::type* = 0) const
    {
        return value;
    }

    template <typename T>
    double asDoubleImpl(typename std::enable_if<!std::is_arithmetic<T>::value>::type* = 0) const
    {
        throw std::runtime_error("type not convertible to double.");
    }

    template <typename T>
    std::string asStringImpl(typename std::enable_if<std::is_base_of<std::string, T>::value>::type* = 0) const
    {
        return value;
    }

    template <typename T>
    std::string asStringImpl(typename std::enable_if<!std::is_base_of<std::string, T>::value>::type* = 0) const
    {
        return universal_to_string(value);
    }

    std::shared_ptr<Clonable> makeEmptyClone() const override
    {
        return std::shared_ptr<Clonable>(new GenericValueMessage<Type>);
    }
    void serialize(SerializationBuffer &data) const override
    {        
        TokenData::serialize(data);
        data << value;
    }
    void deserialize(const SerializationBuffer& data) override
    {        
        TokenData::deserialize(data);
        data >> value;
    }

    Type value;
};


/// TRAITS
template <typename T>
struct type<GenericValueMessage<T> > {
    static std::string name() {
        return std::string("Value<") + type2name(typeid(T)) + ">";
    }
};

template <typename V>
struct MessageContainer<V, false>
{
    typedef GenericValueMessage<V> type;

    static V& access(GenericValueMessage<V>& msg) {
        return msg.value;
    }
    static const V& accessConst(const GenericValueMessage<V>& msg) {
        return msg.value;
    }
};
}

/// CASTING
///

namespace msg
{

template <typename V, typename S>
struct MessageCaster<connection_types::GenericValueMessage<V>, S>
{
    static std::shared_ptr<connection_types::GenericValueMessage<V> const> constcast(const std::shared_ptr<S const>& msg)
    {
        // if we can dynamic cast directly, use that
        if(auto direct = std::dynamic_pointer_cast<connection_types::GenericValueMessage<V> const>(msg)) {
            return direct;
        }

        // if we can cast the message to the value type, create a new message of that type
        if(auto other_base = dynamic_cast<connection_types::ValueMessageBase const*>(msg.get())) {
            return convert<V>(*other_base);
        }
        // otherwise we cannot do anything
        return nullptr;
    }

    static std::shared_ptr<connection_types::GenericValueMessage<V>> cast(const std::shared_ptr<S>& msg)
    {
        // if we can dynamic cast directly, use that
        if(auto direct = std::dynamic_pointer_cast<connection_types::GenericValueMessage<V>>(msg)) {
            return direct;
        }
        // if we can cast the message to the value type, create a new message of that type
        if(auto other_base = dynamic_cast<connection_types::ValueMessageBase const*>(msg.get())) {
            return convert<V>(*other_base);
        }
        // otherwise we cannot do anything
        return nullptr;
    }

private:
    template <typename T>
    static std::shared_ptr<connection_types::GenericValueMessage<V>> convert(const connection_types::ValueMessageBase& in,
                                                                             typename std::enable_if<std::is_arithmetic<T>::value>::type* = 0)
    {
        auto res = std::make_shared<connection_types::GenericValueMessage<V>>();
        if(std::is_integral<T>::value) {
            res->value = static_cast<T>(in.asInteger());
        } else if(std::is_floating_point<T>::value) {
            res->value = static_cast<T>(in.asDouble());
        } else {
            return nullptr;
        }
        return res;
    }

    template <typename T>
    static std::shared_ptr<connection_types::GenericValueMessage<V>> convert(const connection_types::ValueMessageBase& in,
                                                                             typename std::enable_if<std::is_base_of<std::string, T>::value>::type* = 0)
    {
        auto res = std::make_shared<connection_types::GenericValueMessage<V>>();
        res->value = static_cast<T>(in.asString());
        return res;
    }
    template <typename T>
    static std::shared_ptr<connection_types::GenericValueMessage<V>> convert(const connection_types::ValueMessageBase& in,
                                                                             typename std::enable_if<!std::is_base_of<std::string, T>::value &&
                                                                                                     !std::is_arithmetic<T>::value>::type* = 0)
    {
        return nullptr;
    }
};

}
}

/// YAML
namespace YAML {
template<typename T>
struct convert<csapex::connection_types::GenericValueMessage<T> >
{
  static Node encode(const csapex::connection_types::GenericValueMessage<T>& rhs)
  {
      Node node;
      node["value"] = rhs.value;
      return node;
  }

  static bool decode(const Node& node, csapex::connection_types::GenericValueMessage<T>& rhs)
  {
      if(!node.IsMap()) {
          return false;
      }

      rhs.value = node["value"].as<T>();
      return true;
  }
};
}


#endif // GENERIC_VALUE_MESSAGE_H
