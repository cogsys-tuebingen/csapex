/// HEADER
#include <csapex/serialization/snippet.h>

/// PROJECT
#include <csapex/model/tag.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace csapex;

Snippet::Snippet(const std::string &serialized)
    : yaml_(std::make_shared<YAML::Node>(YAML::Load(serialized)))
{

}

Snippet::Snippet(const YAML::Node &yaml)
    : yaml_(std::make_shared<YAML::Node>(yaml))
{

}

Snippet::Snippet(YAML::Node &&yaml)
    : yaml_(std::make_shared<YAML::Node>(std::move(yaml)))
{

}

Snippet::Snippet()
{

}

void Snippet::toYAML(YAML::Node &out) const
{
    out = *yaml_;
}

void Snippet::setName(const std::string& name)
{
    name_ = name;
}

std::string Snippet::getName() const
{
    return name_;
}

void Snippet::setDescription(const std::string& description)
{
    description_ = description;
}

std::string Snippet::getDescription() const
{
    return description_;
}

void Snippet::setTags(const std::vector<TagConstPtr>& tags)
{
    tags_ = tags;
}

std::vector<TagConstPtr> Snippet::getTags() const
{
    if(!tags_.empty()) {
        return tags_;
    } else {
        return { Tag::get("General") };
    }
}


void Snippet::save(const std::string &file) const
{
    YAML::Node exported;
    exported["name"] = name_;
    exported["description"] = description_;

    std::vector<std::string> tag_str(tags_.size());
    for(const TagConstPtr& tag : tags_) {
        tag_str.push_back(tag->getName());
    }
    exported["tags"] = tag_str;

    YAML::Node yaml;
    toYAML(yaml);
    exported["yaml"] = yaml;

    std::ofstream of(file);
    of << exported;
}

Snippet Snippet::load(const std::string &file)
{
    YAML::Node node = YAML::LoadFile(file);
    Snippet res(node["yaml"]);

    if(node["name"].IsDefined()) {
        res.setName(node["name"].as<std::string>());
    }

    if(node["description"].IsDefined()) {
        res.setDescription(node["description"].as<std::string>());
    }

    if(node["tags"].IsDefined()) {
        std::vector<std::string> tags_str = node["tags"].as<std::vector<std::string>>();
        res.tags_.reserve(tags_str.size());
        for(const std::string& tag_s : tags_str) {
            Tag::createIfNotExists(tag_s);
            res.tags_.push_back(Tag::get(tag_s));
        }
    }

    return res;
}
