#ifndef BOOTSTRAP_PLUGIN_H
#define BOOTSTRAP_PLUGIN_H

/// PROJECT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <class_loader/class_loader_register_macro.h>

#define CSAPEX_REGISTER_BOOT(name) \
    CLASS_LOADER_REGISTER_CLASS(name, BootstrapPlugin)

namespace csapex
{
class BootstrapPlugin
{
public:
    virtual ~BootstrapPlugin();

    virtual void boot(csapex::PluginLocator* locator) = 0;
};
}


#endif // BOOTSTRAP_PLUGIN_H
