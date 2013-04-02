#ifndef BG_DB_STRATEGY_H
#define BG_DB_STRATEGY_H

/// COMPONENT
#include "db_strategy.h"

/// FORWARD DECLARATION
class BagDatabase;

/**
 * @brief The BagDatabaseStrategy class
 */
class BagDatabaseStrategy : public DatabaseStrategy
{
public:
    /**
     * @brief BagDatabaseStrategy
     */
    BagDatabaseStrategy();

    /**
     * @brief ~BagDatabaseStrategy
     */
    virtual ~BagDatabaseStrategy();

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
    BagDatabase* bag_db;
};

#endif // BG_DB_STRATEGY_H
