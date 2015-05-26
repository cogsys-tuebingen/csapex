/// HEADER
#include <csapex/factory/generic_node_factory.hpp>

/// PROJECT
#include <csapex/model/tag.h>

using namespace csapex;

std::vector<TagPtr> GenericNodeFactory::stringsToTags(const std::vector<std::string> &strings)
{
    std::vector<TagPtr> tags;
    for(const std::string& name : strings) {
        tags.emplace_back(Tag::get(name));
    }
    return tags;
}
