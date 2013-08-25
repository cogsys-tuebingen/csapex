/// HEADER
#include <csapex/core_plugin.h>

using namespace csapex;

CorePlugin::~CorePlugin()
{

}

void CorePlugin::setName(const std::string& name)
{
    name_ = name;
}

std::string CorePlugin::getName()
{
    return name_;
}

