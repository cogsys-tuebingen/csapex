#ifndef TRAINER_NODE_STATIC_H
#define TRAINER_NODE_STATIC_H

/// COMPONENT
#include "trainer_adapter.h"

/// PROJECT
#include <db_strategy/evaluation_strategy_decorator.h>

/// SYSTEM
#include <string>
#include <vector>

/// FORWARD DECLARATION
class Trainer;

/**
 * @brief The TrainerAdapterStatic class provides its adaptee input images from a directory
 *        and provides testing functions
 */
class TrainerAdapterStatic : public TrainerAdapter
{
public:
    /**
     * @brief TrainerAdapterStatic
     * @param initial config
     * @param trainer The trainer to wrap
     * @param out Output stream onto which resulting statistics will be written
     * @throws if something went wrong
     */
    TrainerAdapterStatic(Trainer& trainer, std::ostream& out);

    /**
     * @brief ~TrainerAdapterStatic
     */
    ~TrainerAdapterStatic();

    /**
     * @brief runHeadless Runs the main loop without a QtWindow
     */
    void runHeadless();

private:
    void import();
    void importNegative();
    void importPositive();
    void importValidation();
    void importTest();

private:
    std::ostream& out;

    std::vector<std::string> training, validation, negative;
    std::vector<std::string> test_positive, test_negative;

    EvaluationStrategyDecorator::DecoPtr db;
};

#endif // TRAINER_NODE_STATIC_H
