#ifndef FACTORY_H
#define FACTORY_H

/// COMPONENT
#include "db_strategy_interface.h"

/// PROJECT
#include <config/config.h>

/// SYSTEM
#include <string>

/**
 * @brief The DatabaseStrategyFactory class is used to create Instances of DatabaseStrategy
 */
class DatabaseStrategyFactory
{
public:
    class IllegalStrategyException : std::exception {};

private:
    /**
     * @brief DatabaseStrategyFactory
     */
    DatabaseStrategyFactory();

public:
    /**
     * @brief create creates a new instance of a DatabaseStrategie
     * @return
     */
    static DatabaseStrategyInterface::Ptr create();
};

#endif // FACTORY_H
