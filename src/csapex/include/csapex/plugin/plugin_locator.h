#ifndef PLUGIN_LOCATOR_H
#define PLUGIN_LOCATOR_H

/// COMPONENT
#include <csapex/core/settings.h>

/// SYSTEM
#include <string>
#include <map>
#include <typeinfo>
#include <vector>
#include <boost/function.hpp>
#include <iostream>
#include <set>

namespace csapex
{
class PluginLocator
{
public:
    PluginLocator(Settings& settings);

    template <typename PluginType>
    std::vector<std::string> enumerateXmlFiles() {
        std::map<const std::type_info*, std::vector<boost::function<void(std::vector<std::string>&)> > >::iterator pos = locators_.find(&typeid(PluginType));
        std::vector<std::string> files;
        if(pos != locators_.end()) {
            std::vector<boost::function<void(std::vector<std::string>&)> >& vec = pos->second;
            for(std::size_t i = 0, total = vec.size(); i < total; ++i) {
                vec.at(i)(files);
            }
        }
        return files;
    }

    template <typename PluginType>
    void registerLocator(boost::function<void(std::vector<std::string>&)> fn)
    {
        locators_[&typeid(PluginType)].push_back(fn);
    }

    void ignoreLibrary(const std::string& name, bool ignore);
    bool isLibraryIgnored(const std::string& name) const;

    void setLibraryError(const std::string& name, const std::string& error);
    bool hasLibraryError(const std::string& name) const;
    std::string getLibraryError(const std::string& name) const;

    void setLibraryLoaded(const std::string& name);
    bool isLibraryLoaded(const std::string& name) const;

    std::vector<std::string> getAllLibraries() const;

private:
    Settings &settings_;

    std::map<const std::type_info*, std::vector<boost::function<void(std::vector<std::string>&)> > > locators_;

    std::set<std::string> loaded_libraries_;
    std::map<std::string, std::string> error_libraries_;

    std::set<std::string> ignored_libraries_;
    param::StringListParameterPtr ignored_persistent_;
};
}

#endif // PLUGIN_LOCATOR_H
