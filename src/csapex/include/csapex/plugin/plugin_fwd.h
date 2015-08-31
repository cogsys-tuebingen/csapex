#ifndef PLUGIN_FWD_H
#define PLUGIN_FWD_H

/// shared_ptr
#include <memory>

#define FWD(name) \
    class name;\
    typedef std::shared_ptr<name> name##Ptr;\
    typedef std::unique_ptr<name> name##UniquePtr;\
    typedef std::weak_ptr<name> name##WeakPtr;\
    typedef std::shared_ptr<const name> name##ConstPtr;


namespace csapex
{
template <typename T>
class PluginManager;

FWD(PluginLocator);
}

#undef FWD

#endif // PLUGIN_FWD_H

