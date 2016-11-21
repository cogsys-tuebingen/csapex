#ifndef SNIPPET_FACTORY_H
#define SNIPPET_FACTORY_H

/// COMPONENT
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/utility/slim_signal.hpp>

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
    void saveSnippet(const Snippet& s, const std::string& path);

    SnippetPtr getSnippetNoThrow(const std::string& name) const noexcept;
    SnippetPtr getSnippet(const std::string& name) const;

    std::map<std::string, SnippetPtr>& getSnippets();

    std::map<std::string, std::vector<SnippetPtr>>& getTagMap();

public:
    slim_signal::Signal<void()> snippet_set_changed;

private:
    void addSnippet(SnippetPtr s);

private:
    PluginLocator* plugin_locator_;

    std::map<std::string, SnippetPtr> constructors_;
    std::map<std::string, std::vector<SnippetPtr>> tag_map_;
};

}

#endif // SNIPPET_FACTORY_H
