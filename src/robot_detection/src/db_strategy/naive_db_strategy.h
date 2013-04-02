#ifndef NAIVE_TRAINING_STRATEGY_H
#define NAIVE_TRAINING_STRATEGY_H

/// COMPONENT
#include "db_strategy.h"

/**
 * @brief The NaiveDatabaseStrategy class encapsulates a NaiveDatabase
 */
class NaiveDatabaseStrategy : public DatabaseStrategy
{
public:
    /**
     * @brief NaiveDatabaseStrategy
     */
    NaiveDatabaseStrategy();

    /**
     * @brief ~NaiveDatabaseStrategy
     */
    virtual ~NaiveDatabaseStrategy();

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
     * @brief validate validate the database using positive and negative examples
     */
    void validate();
};

#endif // NAIVE_TRAINING_STRATEGY_H
