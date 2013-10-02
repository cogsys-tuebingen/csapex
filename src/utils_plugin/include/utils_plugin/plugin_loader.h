#ifndef PLUGIN_LOADED_H
#define PLUGIN_LOADED_H

/// COMPONENT
#include <utils_plugin/singleton.hpp>

/// SYSTEM
#include <class_loader/multi_library_class_loader.h>

class PluginLoader : public Singleton<PluginLoader> {
public:
    PluginLoader();
    ~PluginLoader();

    void load(const std::string& path);
    bool isLoaded(const std::string& path);

    template <class T>
    static boost::shared_ptr<T> createInstance(const std::string& class_name)
    {
        return instance().lowlevel_class_loader_.createInstance<T>(class_name);
    }

    template <class T>
    std::vector<std::string> getAvailableClasses()
    {
        return lowlevel_class_loader_.getAvailableClasses<T>();
    }

    void stop();
private:
    class_loader::MultiLibraryClassLoader lowlevel_class_loader_;
    std::map<std::string, bool> loaded_;
};

#endif // PLUGIN_LOADED_H
