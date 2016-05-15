#ifndef GENERIC_VALUE_MESSAGE_H
#define GENERIC_VALUE_MESSAGE_H

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/utility/register_msg.h>
#include <csapex/serialization/message_serializer.h>

namespace csapex {
namespace connection_types {

template <typename Type>
struct GenericValueMessage : public Message
{
    typedef std::shared_ptr<GenericValueMessage<Type> > Ptr;
    typedef std::shared_ptr<GenericValueMessage<Type> const> ConstPtr;

    GenericValueMessage(const std::string& frame_id = "/", Message::Stamp stamp = 0)
        : Message(type< GenericValueMessage<Type> >::name(), frame_id, stamp)
    {
        static csapex::DirectMessageConstructorRegistered<connection_types::GenericValueMessage, Type> reg_c;
        static csapex::DirectMessageSerializerRegistered<connection_types::GenericValueMessage, Type> reg_s;
    }

    virtual TokenData::Ptr clone() const override
    {
        Ptr new_msg(new GenericValueMessage<Type>(frame_id, stamp_micro_seconds));
        new_msg->value = value;
        return new_msg;
    }

    virtual TokenData::Ptr toType() const override
    {
        Ptr new_msg(new GenericValueMessage<Type>("/"));
        return new_msg;
    }

    bool acceptsConnectionFrom(const TokenData* other_side) const override
    {
        return descriptiveName() == other_side->descriptiveName();
    }

    Type getValue()
    {
        return value;
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
