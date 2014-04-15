#ifndef REGISTER_CORE_PLUGINS_H
#define REGISTER_CORE_PLUGINS_H

/// HEADER
#include <csapex/core/core_plugin.h>

namespace csapex {

class RegisterCorePlugins : public CorePlugin
{
public:
    RegisterCorePlugins();

    void init(CsApexCore& core);
    void initUI(DragIO &dragio);
    void shutdown();
};

}

#endif // REGISTER_CORE_PLUGINS_H
