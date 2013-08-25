#ifndef REGISTER_PLUGIN_H
#define REGISTER_PLUGIN_H

/// HEADER
#include <csapex/core_plugin.h>

namespace csapex {

class RegisterVisionFeaturePlugin : public CorePlugin
{
public:
    RegisterVisionFeaturePlugin();

    void init();
};

}

#endif // REGISTER_PLUGIN_H
