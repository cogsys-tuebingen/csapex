#ifndef BOW_DB_STRATEGY_H
#define BOW_DB_STRATEGY_H

/// COMPONENT
#include "db_strategy.h"

/// FORWARD DECLARATION
class BowDatabase;

/**
 * @brief The BagDatabaseStrategy class
 */
class BowDatabaseStrategy : public DatabaseStrategy
{
public:
    /**
     * @brief BagDatabaseStrategy
     */
    BowDatabaseStrategy();

    /**
     * @brief ~BagDatabaseStrategy
     */
    virtual ~BowDatabaseStrategy();

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
     * @param validation examples
     */
    void validate();

protected:
    BowDatabase* bow_db;
};

#endif // BOW_DB_STRATEGY_H
