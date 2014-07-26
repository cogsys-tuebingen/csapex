#ifndef GENERIC_VALUE_MESSAGE_H
#define GENERIC_VALUE_MESSAGE_H

/// COMPONENT
#include <csapex/msg/message.h>

namespace csapex {
namespace connection_types {

template <typename Type>
struct GenericValueMessage : public Message {
    typedef boost::shared_ptr<GenericValueMessage<Type> > Ptr;

    GenericValueMessage(const std::string& frame_id = "/")
        : Message(type2name(typeid(Type)), frame_id)
    {}

    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new GenericValueMessage<Type>(frame_id));
        new_msg->value = value;
        return new_msg;
    }

    virtual ConnectionType::Ptr toType() {
        Ptr new_msg(new GenericValueMessage<Type>("/"));
        return new_msg;
    }

    static ConnectionType::Ptr make(){
        Ptr new_msg(new GenericValueMessage<Type>("/"));
        return new_msg;
    }

    bool acceptsConnectionFrom(const ConnectionType* other_side) const {
        return name() == other_side->name();
    }

    void writeYaml(YAML::Emitter& yaml) const {
        yaml << YAML::Key << "value" << YAML::Value << value;
    }
    void readYaml(YAML::Node& node) {
    }

    Type getValue() {
        return value;
    }

    Type value;
};

}
}

#endif // GENERIC_VALUE_MESSAGE_H
