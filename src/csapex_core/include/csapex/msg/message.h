#ifndef MESSAGE_H
#define MESSAGE_H

/// COMPONENT
#include <csapex/model/token_data.h>
#include <csapex/utility/type.h>
#include <csapex/msg/token_traits.h>
#include <csapex_core/csapex_core_export.h>
#include <csapex/utility/export_plugin.h>

namespace csapex
{
namespace connection_types
{

class CSAPEX_CORE_EXPORT Message : public TokenData
{
public:
    struct Version
    {
        int major_v = -1;
        int minor_v = -1;
        int patch_v = -1;

        Version(int major, int minor, int patch);

        Version() = default;

        bool operator < (const Version& other);
        bool operator == (const Version& other);
        bool operator > (const Version& other);

        bool valid() const;
        operator bool() const;
    };

public:
    typedef std::shared_ptr<Message> Ptr;
    typedef std::shared_ptr<Message const> ConstPtr;
    typedef std::uint64_t Stamp;

protected:
    Message(const std::string& name, const std::string& frame_id, Stamp stamp_micro_seconds);
    virtual ~Message();

    void cloneDataFrom(const Clonable& other);

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
struct CSAPEX_CORE_EXPORT convert<csapex::connection_types::Message> {
    static Node encode(const csapex::connection_types::Message& rhs,
                       const csapex::connection_types::Message::Version& version = csapex::connection_types::Message::Version(0,0,0));
    static csapex::connection_types::Message::Version decode(const Node& node, csapex::connection_types::Message& rhs);
};
}

#endif // MESSAGE_H
