#ifndef MESSAGE_H
#define MESSAGE_H

/// COMPONENT
#include <csapex/model/connection_type.h>
#include <csapex/utility/type.h>

///
/// FOR OPTIONAL ROS SUPPORT
///
namespace ros
{
namespace message_traits
{

template<typename M>
struct IsMessage;

template<typename M>
const char* md5sum();

template<typename M>
const char* datatype();

template<typename M>
const char* definition();

template<typename M>
bool hasHeader();

}
}

///
///
///



namespace csapex {
namespace connection_types {

struct Message : public ConnectionType
{
public:
    typedef boost::shared_ptr<Message> Ptr;

protected:
    Message(const std::string& name);
    virtual ~Message();

public:
    void writeYaml(YAML::Emitter& yaml);
    void readYaml(const YAML::Node& node);

public:
    std::string frame_id;
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

struct PossibleRosMessage : public Message
{
public:
    typedef boost::shared_ptr<PossibleRosMessage> Ptr;

protected:
    PossibleRosMessage(const std::string& name)
        : Message(name)
    {
    }

public:
    virtual bool isRosMessage() const
    {
        return false;
    }

    virtual void info(std::string& /*md5*/, std::string& /*datatype*/, std::string& /*def*/, bool& /*header*/) {

    }

};

template <typename Type>
struct GenericMessage : public PossibleRosMessage {
    typedef boost::shared_ptr<GenericMessage<Type> > Ptr;

    GenericMessage()
        : PossibleRosMessage(GenericMessage::template getTypeName<Type>())
    {}

    template <typename T>
    static std::string getTypeName(typename boost::enable_if<ros::message_traits::IsMessage<T> >::type* dummy = 0)
    {
        return ros::message_traits::datatype<T>();
    }

    template <typename T>
    static std::string getTypeName(typename boost::disable_if<ros::message_traits::IsMessage<T> >::type* dummy = 0)
    {
        return type2name(typeid(T));
    }

    virtual bool isRosMessage() const
    {
        return isRosMessageImpl<Type>();
    }

    template <typename T>
    bool isRosMessageImpl(typename boost::enable_if<ros::message_traits::IsMessage<T> >::type* dummy = 0) const
    {
        return true;
    }

    template <typename T>
    bool isRosMessageImpl(typename boost::disable_if<ros::message_traits::IsMessage<T> >::type* dummy = 0) const
    {
        return false;
    }

    virtual void info(std::string& md5, std::string& datatype, std::string& def, bool& header) {
        infoImpl<Type>(md5, datatype, def, header);
    }

    template <typename T>
    void infoImpl(std::string& md5, std::string& datatype, std::string& def, bool& header,
                  typename boost::enable_if<ros::message_traits::IsMessage<T> >::type* dummy = 0) {
        md5 = ros::message_traits::md5sum<Type>();
        datatype = ros::message_traits::datatype<Type>();
        def = ros::message_traits::definition<Type>();
        header = ros::message_traits::hasHeader<Type>();
    }

    template <typename T>
    void infoImpl(std::string& md5, std::string& datatype, std::string& def, bool& header,
                  typename boost::disable_if<ros::message_traits::IsMessage<T> >::type* dummy = 0) {
    }

    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new GenericMessage<Type>);
        new_msg->value = value;
        return new_msg;
    }

    virtual ConnectionType::Ptr toType() {
        Ptr new_msg(new GenericMessage<Type>);
        return new_msg;
    }

    static ConnectionType::Ptr make(){
        Ptr new_msg(new GenericMessage<Type>);
        return new_msg;
    }

    bool acceptsConnectionFrom(const ConnectionType* other_side) const {
        return name() == other_side->name();
    }

    void writeYaml(YAML::Emitter& yaml) {
        yaml << YAML::Key << "value" << YAML::Value << "not implemented";
    }
    void readYaml(YAML::Node& node) {
    }

    typename boost::shared_ptr<Type> value;
};

template <typename Type>
struct DirectMessage : public Message {
    typedef boost::shared_ptr<DirectMessage<Type> > Ptr;

    DirectMessage()
        : Message(type2name(typeid(Type)))
    {}

    virtual bool isRosMessage() const
    {
        return false;
    }

    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new DirectMessage<Type>);
        new_msg->value = value;
        return new_msg;
    }

    virtual ConnectionType::Ptr toType() {
        Ptr new_msg(new DirectMessage<Type>);
        return new_msg;
    }

    static ConnectionType::Ptr make(){
        Ptr new_msg(new DirectMessage<Type>);
        return new_msg;
    }

    bool acceptsConnectionFrom(const ConnectionType* other_side) const {
        return name() == other_side->name();
    }

    void writeYaml(YAML::Emitter& yaml) {
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

    MessageTemplate(const std::string& name)
        : Message(name)
    {}

    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new Instance);
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

    void writeYaml(YAML::Emitter& yaml) {
        yaml << YAML::Key << "value" << YAML::Value << "not implemented";
    }
    void readYaml(YAML::Node& node) {
    }

    Type value;
};


}
}

#endif // MESSAGE_H
