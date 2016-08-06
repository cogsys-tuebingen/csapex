#ifndef MESSAGE_H
#define MESSAGE_H

/// COMPONENT
#include <csapex/model/token_data.h>
#include <csapex/utility/type.h>
//#include <csapex/utility/yaml_io.hpp> // TODO: get rid of this....
#include <csapex/msg/token_traits.h>

namespace csapex
{
namespace connection_types
{

class Message : public TokenData
{
public:
    typedef std::shared_ptr<Message> Ptr;
    typedef std::shared_ptr<Message const> ConstPtr;
    typedef u_int64_t Stamp;

protected:
    Message(const std::string& name, const std::string& frame_id, Stamp stamp_micro_seconds);
    virtual ~Message();

public:
    std::string frame_id;
    Stamp stamp_micro_seconds;
};

}
}

/// YAML
namespace YAML {
class Node;
template<class T>
struct convert;

template<>
struct convert<csapex::connection_types::Message> {
  static Node encode(const csapex::connection_types::Message& rhs);
  static bool decode(const Node& node, csapex::connection_types::Message& rhs);
};
}

#endif // MESSAGE_H
