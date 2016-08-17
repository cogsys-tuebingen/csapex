#ifndef BOOTSTRAP_PLUGIN_H
#define BOOTSTRAP_PLUGIN_H

/// PROJECT
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/csapex_export.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/utility/export_plugin.h>

#define CSAPEX_REGISTER_BOOT(name) \
    CLASS_LOADER_REGISTER_CLASS_WITH_MESSAGE(name, BootstrapPlugin, "INSTALLING BOOTSTRAP PLUGIN")

namespace csapex
{
class CSAPEX_EXPORT BootstrapPlugin
{
public:
    virtual ~BootstrapPlugin();

    virtual void boot(csapex::PluginLocator* locator) = 0;
};
}


#endif // BOOTSTRAP_PLUGIN_H
