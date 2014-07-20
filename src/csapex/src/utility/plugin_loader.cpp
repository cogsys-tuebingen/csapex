/// HEADER
#include <csapex/utility/plugin_loader.h>

/// SYSTEM
#include <boost/foreach.hpp>

PluginLoader::PluginLoader()
    : lowlevel_class_loader_(false)
{
}

void PluginLoader::stop()
{
    typedef std::pair<std::string, bool> PAIR;
    BOOST_FOREACH(const PAIR& p, loaded_) {
        if(p.second) {
            lowlevel_class_loader_.unloadLibrary(p.first);
        }
    }
}

PluginLoader::~PluginLoader()
{
    try {
        stop();
    } catch(...) {

    }
}

bool PluginLoader::isLoaded(const std::string &path)
{
    return loaded_[path];
}

void PluginLoader::load(const std::string &path)
{
    lowlevel_class_loader_.loadLibrary(path);
    loaded_[path] = true;
}
