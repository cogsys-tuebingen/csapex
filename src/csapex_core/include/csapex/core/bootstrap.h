#ifndef BOOTSTRAP_H
#define BOOTSTRAP_H

/// PROJECT
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/command/dispatcher.h>
#include <csapex_core/csapex_core_export.h>

namespace class_loader
{
class ClassLoader;
}

namespace csapex
{
class CSAPEX_CORE_EXPORT Bootstrap
{
public:
    Bootstrap();
    ~Bootstrap();

    bool bootFrom(const std::string& directory, PluginLocator* plugin_locator, bool require_boot_plugin = true);

private:
    bool tryBootFrom(const std::string& directory, PluginLocator* plugin_locator, bool require_boot_plugin);

private:
    std::vector<class_loader::ClassLoader*> boot_plugin_loaders_;
    std::vector<BootstrapPluginPtr> boot_plugins_;
};

}  // namespace csapex

#endif  // BOOTSTRAP_H
