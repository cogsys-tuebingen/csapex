#ifndef SNIPPET_H
#define SNIPPET_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/serialization/serialization_fwd.h>

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

class Snippet
{
public:
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

private:
    mutable std::shared_ptr<YAML::Node> yaml_;

    std::string name_;
    std::string description_;
    std::vector<TagConstPtr> tags_;
};

}

#endif // SNIPPET_H
