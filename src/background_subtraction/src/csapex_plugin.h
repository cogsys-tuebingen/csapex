#ifndef CSAPEX_PLUGIN_H
#define CSAPEX_PLUGIN_H

/// HEADER
#include <csapex/core_plugin.h>

namespace csapex {

class CSAPEXPlugin : public CorePlugin
{
public:
    CSAPEXPlugin();

    void init();
};

}

#endif // CSAPEX_PLUGIN_H
