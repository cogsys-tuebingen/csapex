/// HEADER
#include "factory.h"

/// COMPONENT
#include "bag_db_strategy.h"
#include "bow_db_strategy.h"
#include "bin_db_strategy.h"
#include "db_strategy.h"
#include "naive_db_strategy.h"

/// SYSTEM
#include <iostream>

DatabaseStrategyFactory::DatabaseStrategyFactory()
{
}

DatabaseStrategyInterface::Ptr DatabaseStrategyFactory::create()
{
    Config cfg = Config::instance();

    int type = cfg("db_type");
    switch(type) {
    case Types::Strategy::BIN:
        return DatabaseStrategyInterface::Ptr(new BinDatabaseStrategy);

    case Types::Strategy::NAIVE:
        return DatabaseStrategyInterface::Ptr(new NaiveDatabaseStrategy);

#ifdef WITH_DBOW2
    case Types::Strategy::BAG:
        return DatabaseStrategyInterface::Ptr(new BagDatabaseStrategy);
#endif

    case Types::Strategy::BOW:
        return DatabaseStrategyInterface::Ptr(new BowDatabaseStrategy);

    default:
        ERROR("unknown db strategy.");
        throw IllegalStrategyException();
    }
}
