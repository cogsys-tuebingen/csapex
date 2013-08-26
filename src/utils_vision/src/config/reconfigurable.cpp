/// HEADER
#include "reconfigurable.h"

/// PROJECT
#include <data/matchable_pose.h>

Reconfigurable::Reconfigurable()
    : config(Config::getGlobal()), tools(instance().latest_tools)
{
    instance().g_tools_replaced.connect(tools_replaced);
    connection = tools_replaced.connect(boost::bind(&Reconfigurable::applyConfig, this, _1));
}

Reconfigurable::Reconfigurable(const Reconfigurable& rhs)
    : config(rhs.config), tools(rhs.tools)
{
    instance().g_tools_replaced.connect(tools_replaced);
    connection = tools_replaced.connect(boost::bind(&Reconfigurable::applyConfig, this, _1));
}

Reconfigurable& Reconfigurable::operator = (const Reconfigurable& rhs)
{
    if(this == &rhs) {
        return *this;
    }

    config = rhs.config;
    tools = rhs.tools;

    if(connection.connected()) {
        connection.disconnect();
    }
    connection = Config::replace.connect(boost::bind(&Reconfigurable::applyConfig, this, _1));

    return *this;
}

Reconfigurable::~Reconfigurable()
{
    if(connection.connected()) {
        connection.disconnect();
    }
}

void Reconfigurable::replaceTools(const Config& cfg)
{
    MatchablePose::IMAGE_PATH = cfg.db_imgs;

    instance().latest_tools = ConfiguratedTools::create(cfg);
    instance().g_tools_replaced(cfg);
}

void Reconfigurable::applyConfig(const Config& cfg)
{
    config = cfg;
    tools = instance().latest_tools;

    configChanged();
}

Config Reconfigurable::getConfig()
{
    return config;
}

Reconfigurable::Manager::Manager()
{
    tools_updater = Config::replace.connect(&Reconfigurable::replaceTools);
    latest_tools = ConfiguratedTools::create(Config::getGlobal());
}

Reconfigurable::Manager& Reconfigurable::instance()
{
    static Manager m;
    return m;
}
