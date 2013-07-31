#ifndef RECONFIGURABLE_H
#define RECONFIGURABLE_H

/// COMPONENT
#include "config.h"
#include "configurated_tools.h"

/// PROJECT
#include <common/global.hpp>

/// SYSTEM
#include <boost/signals2.hpp>

/**
 * @brief The Reconfigurable class is an interface for classes that use a config
 */
class Reconfigurable
{
public:
    /**
     * @brief Reconfigurable
     */
    Reconfigurable();

    /**
     * @brief Reconfigurable Copy Constructor
     * @param rhs
     */
    Reconfigurable(const Reconfigurable& rhs);

    /**
     * @brief operator = Assignment operator
     * @param rhs
     * @return
     */
    Reconfigurable& operator= (const Reconfigurable& rhs);

    /**
     * @brief ~Reconfigurable
     */
    virtual ~Reconfigurable();

    /**
     * @brief applyConfig is the callback function to apply a new config
     * @param cfg the config to use
     */
    void applyConfig(const Config& cfg);

    /**
     * @brief getConfig Accessor
     * @return the current configuration
     */
    Config getConfig();

protected:
    static void replaceTools(const Config& cfg);
    static boost::signals2::connection tools_updater;
    static boost::signals2::signal<void(const Config&)> tools_replaced;

protected:
    /**
     * @brief configChanged Callback when the config has changed
     */
    virtual void configChanged() {}

protected:
    Config config;
    ConfiguratedTools::Ptr tools;

private:
    static ConfiguratedTools::Ptr latest_tools;
    boost::signals2::connection connection;
};

#endif // RECONFIGURABLE_H
