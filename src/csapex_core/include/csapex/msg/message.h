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
    typedef std::shared_ptr<Message> Ptr;
    typedef std::shared_ptr<Message const> ConstPtr;
    typedef std::uint64_t Stamp;

public:
    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    Message(const std::string& name, const std::string& frame_id, Stamp stamp_micro_seconds);
    ~Message() override;

    bool cloneDataFrom(const Clonable& other) override;

public:
    std::string frame_id;
    Stamp stamp_micro_seconds;
};

}  // namespace connection_types
}  // namespace csapex

/// YAML
namespace YAML
{
class Node;
template <class T>
struct convert;

template <>
struct CSAPEX_CORE_EXPORT convert<csapex::connection_types::Message>
{
    static Node encode(const csapex::connection_types::Message& rhs, const csapex::SemanticVersion& version = csapex::SemanticVersion(0, 0, 0));
    static csapex::SemanticVersion decode(const Node& node, csapex::connection_types::Message& rhs);
};
}  // namespace YAML

#endif  // MESSAGE_H
