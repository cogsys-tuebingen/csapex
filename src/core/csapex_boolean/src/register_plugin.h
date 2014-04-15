#ifndef REGISTER_PLUGIN_H
#define REGISTER_PLUGIN_H

/// HEADER
#include <csapex/core/core_plugin.h>

namespace csapex {

namespace boolean {

class RegisterPlugin : public CorePlugin
{
public:
    RegisterPlugin();

    void init(CsApexCore& core);
};

}

}

#endif // REGISTER_PLUGIN_H
