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
#include <iostream>
#include <set>
#include <boost/signals2.hpp>

class QFileSystemWatcher;
namespace qt_helper
{
class Call;
}

namespace csapex
{
class PluginLocator : private boost::noncopyable
{
public:
    boost::signals2::signal<void(std::string)> unload_library_request;
    boost::signals2::signal<void(std::string)> reload_library_request;
    boost::signals2::signal<void()> reload_done;

public:
    PluginLocator(Settings& settings);
    ~PluginLocator();

    template <typename PluginType>
    std::vector<std::string> enumerateXmlFiles() {
        std::map<const std::type_info*, std::vector<std::function<void(std::vector<std::string>&)> > >::iterator pos = locators_.find(&typeid(PluginType));
        std::vector<std::string> files;
        if(pos != locators_.end()) {
            std::vector<std::function<void(std::vector<std::string>&)> >& vec = pos->second;
            for(std::size_t i = 0, total = vec.size(); i < total; ++i) {
                vec.at(i)(files);
            }
        }
        return files;
    }

    std::vector<std::string> enumerateLibraryPaths();

    template <typename PluginType>
    void registerLocator(std::function<void(std::vector<std::string>&)> fn)
    {
        locators_[&typeid(PluginType)].push_back(fn);
    }

    void reloadLibraryIfExists(const std::string& name, const std::string& abs_path);
    void reloadLibrary(const std::string& name);

    void ignoreLibrary(const std::string& name, bool ignore);
    bool isLibraryIgnored(const std::string& name) const;

    void setLibraryError(const std::string& name, const std::string& error);
    bool hasLibraryError(const std::string& name) const;
    std::string getLibraryError(const std::string& name) const;

    void setLibraryLoaded(const std::string& name, const std::string &file);
    bool isLibraryLoaded(const std::string& name) const;

    void setAutoReload(bool autoreload);
    bool isAutoReload() const;

    std::vector<std::string> getAllLibraries() const;

private:
    void createFileWatcher(const std::string &name);
    void clearFileWatcherCallbacks();

private:
    Settings &settings_;

    std::map<const std::type_info*, std::vector<std::function<void(std::vector<std::string>&)> > > locators_;

    std::vector<std::string> library_paths_;

    std::set<std::string> loaded_libraries_;
    std::map<std::string, std::string> library_file_;
    std::map<std::string, std::shared_ptr<QFileSystemWatcher> > library_watchers_;
    std::map<std::string, std::string> error_libraries_;

    std::set<std::string> ignored_libraries_;
    param::StringListParameterPtr ignored_persistent_;

    std::vector<qt_helper::Call*> callbacks_;
};
}

#endif // PLUGIN_LOCATOR_H
