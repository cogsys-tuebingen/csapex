#ifndef BIN_DB_STRATEGY_H
#define BIN_DB_STRATEGY_H

/// COMPONENT
#include "db_strategy.h"

/// FORWARD DECLARATION
class BinDatabase;

/**
 * @brief The BinDatabaseStrategy class
 */
class BinDatabaseStrategy : public DatabaseStrategy
{
public:
    /**
     * @brief BinDatabaseStrategy
     */
    BinDatabaseStrategy();

    /**
     * @brief ~BinDatabaseStrategy
     */
    virtual ~BinDatabaseStrategy();

    /**
     * @brief train Callback whenever there is a new Frame
     * @param frame the frame to handle
     */
    virtual void train(Frame::Ptr frame);

    /**
     * @brief addValidationExample Add a frame to the validation examples
     * @param frame the frame to analyze
     */
    void addValidationExample(Frame::Ptr frame);

    /**
     * @brief validate train the database using positive and negative examples
     */
    void validate();

protected:
    BinDatabase* bin_db;
};

#endif // BIN_DB_STRATEGY_H
