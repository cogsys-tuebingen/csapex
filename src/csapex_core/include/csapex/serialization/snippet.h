#ifndef SNIPPET_H
#define SNIPPET_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/serialization/streamable.h>

/// SYSTEM
#include <string>
#include <boost/variant.hpp>
#include <memory>
#include <vector>

namespace YAML
{
class Node;
}

namespace csapex
{
class Snippet : public Streamable
{
protected:
    CLONABLE_IMPLEMENTATION(Snippet);

public:
    static const uint8_t PACKET_TYPE_ID = 128;

    Snippet(const std::string& serialized);
    Snippet(const YAML::Node& yaml);
    Snippet(YAML::Node&& yaml);
    Snippet();

    void save(const std::string& file) const;
    static Snippet load(const std::string& file);

    void setName(const std::string& name);
    std::string getName() const;

    void setDescription(const std::string& description);
    std::string getDescription() const;

    void setTags(const std::vector<TagConstPtr>& tags);
    std::vector<TagConstPtr> getTags() const;

    void toYAML(YAML::Node& out) const;

    uint8_t getPacketType() const override;

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    static std::shared_ptr<Snippet> makeEmpty();

private:
    mutable std::shared_ptr<YAML::Node> yaml_;

    std::string name_;
    std::string description_;
    std::vector<TagConstPtr> tags_;
};

}  // namespace csapex

#endif  // SNIPPET_H
