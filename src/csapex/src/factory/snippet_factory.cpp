/// HEADER
#include <csapex/factory/snippet_factory.h>

/// PROJECT
#include <csapex/plugin/plugin_locator.h>
#include <csapex/serialization/snippet.h>
#include <csapex/model/tag.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <iostream>

using namespace csapex;

SnippetFactory::SnippetFactory(PluginLocator *locator)
    : plugin_locator_(locator)
{

}

SnippetFactory::~SnippetFactory()
{

}

std::map<std::string, std::vector<SnippetPtr>>& SnippetFactory::getTagMap()
{
    return tag_map_;
}

void SnippetFactory::addSnippet(SnippetPtr s)
{
    constructors_.insert(std::make_pair(s->getName(), s));

    for(const TagConstPtr& tag : s->getTags()) {
        tag_map_[tag->getName()].push_back(s);
    }
}

void SnippetFactory::loadSnippets()
{
    for(const std::string& dir_string : plugin_locator_->getPluginPaths("snippets")) {
        boost::filesystem::path directory(dir_string);

        if (!boost::filesystem::exists(directory)) {
            std::cerr << "cannot load snippets from " << dir_string << ", the path doesn't exist" << std::endl;
            continue;
        }

        boost::filesystem::directory_iterator dir(directory);
        boost::filesystem::directory_iterator end;

        for(; dir != end; ++dir) {
            boost::filesystem::path path = dir->path();

            if(path.extension() == Settings::template_extension) {
                SnippetPtr s =  std::make_shared<Snippet>(Snippet::load(path.string()));
                addSnippet(s);
            }
        }
    }
}

void SnippetFactory::saveSnippet(const Snippet &s, const std::string &path)
{
    s.save(path);

    if(SnippetPtr existing = getSnippetNoThrow(s.getName())) {
        *existing = s;
    } else {
        addSnippet(std::make_shared<Snippet>(s));
    }

    snippet_set_changed();
}

std::map<std::string, SnippetPtr>& SnippetFactory::getSnippets()
{
    return constructors_;
}

SnippetPtr SnippetFactory::getSnippetNoThrow(const std::string& name) const noexcept
{
    auto pos = constructors_.find(name);
    if(pos == constructors_.end()) {
        return nullptr;
    } else {
        return pos->second;
    }
}

SnippetPtr SnippetFactory::getSnippet(const std::string &name) const
{
    return constructors_.at(name);
}
