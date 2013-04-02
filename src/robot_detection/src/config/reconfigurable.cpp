/// HEADER
#include "reconfigurable.h"

/// PROJECT
#include <data/matchable_pose.h>

boost::shared_ptr<ConfiguratedTools> Reconfigurable::latest_tools(ConfiguratedTools::create(Config::getGlobal()));
boost::signals2::connection Reconfigurable::tools_updater = Config::replace.connect(&Reconfigurable::replaceTools);
boost::signals2::signal<void(const Config&)> Reconfigurable::tools_replaced;

Reconfigurable::Reconfigurable()
    : config(Config::latest), tools(latest_tools)
{
    connection = tools_replaced.connect(boost::bind(&Reconfigurable::applyConfig, this, _1));
}

Reconfigurable::Reconfigurable(const Reconfigurable& rhs)
    : config(rhs.config), tools(rhs.tools)
{
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

    latest_tools = ConfiguratedTools::create(cfg);
    tools_replaced(cfg);
}

void Reconfigurable::applyConfig(const Config& cfg)
{
    config = cfg;
    tools = latest_tools;

    configChanged();
}

Config Reconfigurable::getConfig()
{
    return config;
}
