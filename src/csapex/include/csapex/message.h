#ifndef MESSAGE_H
#define MESSAGE_H

/// COMPONENT
#include <csapex/connection_type.h>

namespace csapex {
namespace connection_types {

struct Message : public ConnectionType
{
public:
    typedef boost::shared_ptr<Message> Ptr;

protected:
    Message(const std::string& name);

public:
    void writeYaml(YAML::Emitter& yaml);
    void readYaml(YAML::Node& node);
};

struct AnyMessage : public Message
{
public:
    typedef boost::shared_ptr<AnyMessage> Ptr;

protected:
    AnyMessage(const std::string& name);

public:
    virtual ConnectionType::Ptr clone() ;
    virtual ConnectionType::Ptr toType();

    static ConnectionType::Ptr make();

    bool canConnectTo(Ptr other_side);
    bool acceptsConnectionFrom(ConnectionType* other_side);
};

}
}

#endif // MESSAGE_H
