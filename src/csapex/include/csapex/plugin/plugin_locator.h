#ifndef PLUGIN_LOCATOR_H
#define PLUGIN_LOCATOR_H

/// COMPONENT
#include <csapex/core/settings.h>

/// SYSTEM
#include <string>
#include <map>
#include <typeinfo>
#include <vector>
#include <functional>
#include <set>
#include <csapex/utility/slim_signal.hpp>
#include <typeindex>

namespace csapex
{
class CSAPEX_EXPORT PluginLocator
{
public:
    PluginLocator(Settings& settings);
    ~PluginLocator();

    void shutdown();

    template <typename PluginType>
    std::vector<std::string> enumerateXmlFiles() {
        std::vector<std::string> files;
        for(auto function : locators_[std::type_index(typeid(PluginType))]) {
            function(files);
        }
        return files;
    }

    std::vector<std::string> enumerateLibraryPaths();

    template <typename PluginType>
    void registerLocator(std::function<void(std::vector<std::string>&)> fn)
    {
        locators_[std::type_index(typeid(PluginType))].push_back(fn);
    }

    void ignoreLibrary(const std::string& name, bool ignore);
    bool isLibraryIgnored(const std::string& name) const;

    void setLibraryError(const std::string& name, const std::string& error);
    bool hasLibraryError(const std::string& name) const;
    std::string getLibraryError(const std::string& name) const;

    void setLibraryLoaded(const std::string& name, const std::string &file);
    bool isLibraryLoaded(const std::string& name) const;

    std::vector<std::string> getAllLibraries() const;

    void setPluginPaths(const std::string& type, const std::vector<std::string>& paths);
    std::vector<std::string> getPluginPaths(const std::string& type) const;

private:
    PluginLocator(const PluginLocator& copy) = delete;
    PluginLocator& operator = (const PluginLocator& copy) = delete;

private:
    Settings &settings_;

    std::map<std::type_index, std::vector<std::function<void(std::vector<std::string>&)> > > locators_;

    std::vector<std::string> library_paths_;

    std::set<std::string> loaded_libraries_;
    std::map<std::string, std::string> library_file_;
    std::map<std::string, std::string> error_libraries_;

    std::set<std::string> ignored_libraries_;
    param::StringListParameterPtr ignored_persistent_;

    std::map<std::string, std::vector<std::string>> plugin_paths_;
};
}

#endif // PLUGIN_LOCATOR_H
