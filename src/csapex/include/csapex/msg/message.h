#ifndef MESSAGE_H
#define MESSAGE_H

/// COMPONENT
#include <csapex/model/token_data.h>
#include <csapex/utility/type.h>
#include <csapex/msg/token_traits.h>
#include <csapex/csapex_export.h>
#include <csapex/utility/export_plugin.h>

namespace csapex
{
namespace connection_types
{

class CSAPEX_EXPORT Message : public TokenData
{
public:
    typedef std::shared_ptr<Message> Ptr;
    typedef std::shared_ptr<Message const> ConstPtr;
    typedef std::uint64_t Stamp;

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
struct CSAPEX_EXPORT convert<csapex::connection_types::Message> {
  static Node encode(const csapex::connection_types::Message& rhs);
  static bool decode(const Node& node, csapex::connection_types::Message& rhs);
};
}

#endif // MESSAGE_H
