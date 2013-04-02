/// HEADER
#include "db_strategy_interface.h"

DatabaseStrategyInterface::DatabaseStrategyInterface(Database* db)
    : db(db)
{
}

DatabaseStrategyInterface::DatabaseStrategyInterface(DatabaseStrategyInterface& copy)
    : Reconfigurable(copy), db(copy.db)
{
}

DatabaseStrategyInterface::~DatabaseStrategyInterface()
{
}
