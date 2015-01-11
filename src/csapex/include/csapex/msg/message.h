#ifndef MESSAGE_H
#define MESSAGE_H

/// COMPONENT
#include <csapex/model/connection_type.h>
#include <csapex/utility/type.h>
#include <csapex/utility/yaml_io.hpp>
#include <csapex/msg/message_traits.h>

namespace csapex {
namespace connection_types {

/// DEFAULT TYPES
struct Message : public ConnectionType
{
public:
    typedef boost::shared_ptr<Message> Ptr;
    typedef boost::shared_ptr<Message const> ConstPtr;
    typedef u_int64_t Stamp;

protected:
    Message(const std::string& name, const std::string& frame_id, Stamp stamp);
    virtual ~Message();

public:
    std::string frame_id;
    Stamp stamp;
};

struct NoMessage : public Message
{
public:
    typedef boost::shared_ptr<NoMessage> Ptr;

public:
    NoMessage();

public:
    virtual ConnectionType::Ptr clone() const override;
    virtual ConnectionType::Ptr toType() const override;

    bool canConnectTo(const ConnectionType* other_side) const override;
    bool acceptsConnectionFrom(const ConnectionType* other_side) const override;
};
template <>
struct type<NoMessage> {
    static std::string name() {
        return "Nothing";
    }
};

struct AnyMessage : public Message
{
public:
    typedef boost::shared_ptr<AnyMessage> Ptr;

public:
    AnyMessage();

public:
    virtual ConnectionType::Ptr clone() const override;
    virtual ConnectionType::Ptr toType() const override;

    bool canConnectTo(const ConnectionType* other_side) const override;
    bool acceptsConnectionFrom(const ConnectionType* other_side) const override;
};
template <>
struct type<AnyMessage> {
    static std::string name() {
        return "Anything";
    }
};
}
}

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::AnyMessage> {
  static Node encode(const csapex::connection_types::AnyMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::AnyMessage& rhs);
};

template<>
struct convert<csapex::connection_types::Message> {
  static Node encode(const csapex::connection_types::Message& rhs);
  static bool decode(const Node& node, csapex::connection_types::Message& rhs);
};
}

#endif // MESSAGE_H
