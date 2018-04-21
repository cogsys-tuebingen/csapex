#ifndef GENERIC_POINTER_MESSAGE_H
#define GENERIC_POINTER_MESSAGE_H

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/serialization/message_serializer.h>
#include <csapex/utility/register_msg.h>
#include <csapex/utility/shared_ptr_tools.hpp>

namespace csapex {
namespace connection_types {

template <typename Type>
struct GenericPointerMessage : public Message
{
protected:
    CLONABLE_IMPLEMENTATION(GenericPointerMessage<Type>);

public:
    typedef std::shared_ptr<GenericPointerMessage<Type> > Ptr;
    typedef std::shared_ptr<GenericPointerMessage<Type> const> ConstPtr;

    GenericPointerMessage(const std::string& frame_id = "/", Message::Stamp stamp = 0)
        : Message(type<GenericPointerMessage<Type>>::name(), frame_id, stamp)
    {
        static csapex::DirectMessageConstructorRegistered<connection_types::GenericPointerMessage, Type> reg_c;
        static csapex::DirectMessageSerializerRegistered<connection_types::GenericPointerMessage, Type> reg_s;

        setDescriptiveName(type2name(typeid(Type)));
    }
    GenericPointerMessage(const std::shared_ptr<Type>& ptr, const std::string& frame_id = "/", Message::Stamp stamp = 0)
        : GenericPointerMessage(frame_id, stamp)
    {
        value = ptr;
    }

    bool acceptsConnectionFrom(const TokenData* other_side) const override
    {
        return descriptiveName() == other_side->descriptiveName();
    }

    typename std::shared_ptr<Type> value;
};


/// TRAITS
template <typename T>
struct type<GenericPointerMessage<T> > {
    static std::string name() {
        return std::string("Pointer<") + type2name(typeid(T)) + ">";
    }
};

}
}

/// YAML
namespace YAML {
template<typename T>
struct convert<csapex::connection_types::GenericPointerMessage<T> > {
  static Node encode(const csapex::connection_types::GenericPointerMessage<T>& rhs) {
      Node node;
      node["value"] = *rhs.value;
      return node;
  }

  static bool decode(const Node& node, csapex::connection_types::GenericPointerMessage<T>& rhs) {
      if(!node.IsMap()) {
          return false;
      }

      rhs.value.reset(new T);
      (*rhs.value) = node["value"].as<T>();
      return true;
  }
};
}


#endif // GENERIC_POINTER_MESSAGE_H
