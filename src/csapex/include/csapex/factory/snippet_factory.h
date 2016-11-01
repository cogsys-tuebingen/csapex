#ifndef SNIPPET_FACTORY_H
#define SNIPPET_FACTORY_H

/// COMPONENT
#include <csapex/serialization/serialization_fwd.h>

/// SYSTEM
#include <string>
#include <functional>
#include <map>
#include <memory>
#include <vector>

namespace csapex
{

class PluginLocator;

class SnippetFactory
{
public:
    SnippetFactory(PluginLocator *locator);
    ~SnippetFactory();

    void loadSnippets();

    SnippetPtr getSnippet(const std::string& name) const;

    std::map<std::string, SnippetPtr>& getSnippets();

    std::map<std::string, std::vector<SnippetPtr>>& getTagMap();

private:
    PluginLocator* plugin_locator_;

    std::map<std::string, SnippetPtr> constructors_;
    std::map<std::string, std::vector<SnippetPtr>> tag_map_;
};

}

#endif // SNIPPET_FACTORY_H
