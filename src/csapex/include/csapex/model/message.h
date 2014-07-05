#ifndef MESSAGE_H
#define MESSAGE_H

/// COMPONENT
#include <csapex/model/connection_type.h>
#include <csapex/utility/type.h>





namespace csapex {
namespace connection_types {

/// TRAITS
template <typename T>
struct type {
    //std::string name();
};


/// DEFAULT TYPES
struct Message : public ConnectionType
{
public:
    typedef boost::shared_ptr<Message> Ptr;

protected:
    Message(const std::string& name, const std::string& frame_id);
    virtual ~Message();

public:
    void writeYaml(YAML::Emitter& yaml) const;
    void readYaml(const YAML::Node& node);

public:
    std::string frame_id;
};

struct NoMessage : public Message
{
public:
    typedef boost::shared_ptr<NoMessage> Ptr;

protected:
    NoMessage();

public:
    virtual ConnectionType::Ptr clone() ;
    virtual ConnectionType::Ptr toType();

    static ConnectionType::Ptr make();

    bool canConnectTo(const ConnectionType* other_side) const;
    bool acceptsConnectionFrom(const ConnectionType* other_side) const;
};

struct AnyMessage : public Message
{
public:
    typedef boost::shared_ptr<AnyMessage> Ptr;

protected:
    AnyMessage();

public:
    virtual ConnectionType::Ptr clone() ;
    virtual ConnectionType::Ptr toType();

    static ConnectionType::Ptr make();

    bool canConnectTo(const ConnectionType* other_side) const;
    bool acceptsConnectionFrom(const ConnectionType* other_side) const;
};


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

    static ConnectionType::Ptr make(){
        Ptr new_msg(new GenericPointerMessage<Type>("/"));
        return new_msg;
    }

    bool acceptsConnectionFrom(const ConnectionType* other_side) const {
        return name() == other_side->name();
    }

    void writeYaml(YAML::Emitter& yaml) const {
        yaml << YAML::Key << "value" << YAML::Value << "not implemented";
    }
    void readYaml(YAML::Node& node) {
    }

    typename boost::shared_ptr<Type> value;
};

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

template <typename Type, class Instance>
struct MessageTemplate : public Message {
    typedef boost::shared_ptr<Instance> Ptr;

    MessageTemplate( const std::string& frame_id = "/")
        : Message(type<Instance>::name(), frame_id)
    {}

    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new Instance);
        new_msg->frame_id = frame_id;
        new_msg->value = value;
        return new_msg;
    }

    virtual ConnectionType::Ptr toType() {
        Ptr new_msg(new Instance);
        return new_msg;
    }

    static ConnectionType::Ptr make(){
        Ptr new_msg(new Instance);
        return new_msg;
    }

    bool acceptsConnectionFrom(const ConnectionType* other_side) const {
        return name() == other_side->name();
    }

    void writeYaml(YAML::Emitter& yaml) const {
        yaml << YAML::Key << "value" << YAML::Value << "not implemented";
    }
    void readYaml(YAML::Node& node) {
    }

    Type value;
};


}
}

#endif // MESSAGE_H
