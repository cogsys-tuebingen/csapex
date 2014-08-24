#ifndef GENERIC_POINTER_MESSAGE_H
#define GENERIC_POINTER_MESSAGE_H

/// COMPONENT
#include <csapex/msg/message.h>

namespace csapex {
namespace connection_types {

template <typename Type>
struct GenericPointerMessage : public Message {
    typedef boost::shared_ptr<GenericPointerMessage<Type> > Ptr;

    GenericPointerMessage(const std::string& frame_id = "/")
        : Message(type2name(typeid(Type)), frame_id)
    {}

    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new GenericPointerMessage<Type>(frame_id));
        new_msg->value = value;
        return new_msg;
    }

    virtual ConnectionType::Ptr toType() {
        Ptr new_msg(new GenericPointerMessage<Type>(frame_id));
        return new_msg;
    }

    bool acceptsConnectionFrom(const ConnectionType* other_side) const {
        return name() == other_side->name();
    }

    typename boost::shared_ptr<Type> value;
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
