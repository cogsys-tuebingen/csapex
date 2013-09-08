#ifndef REGISTER_PLUGIN_H
#define REGISTER_PLUGIN_H

/// HEADER
#include <csapex/core_plugin.h>

namespace csapex {

namespace boolean {

class RegisterPlugin : public CorePlugin
{
public:
    RegisterPlugin();

    void init();
};

}

}

#endif // REGISTER_PLUGIN_H
