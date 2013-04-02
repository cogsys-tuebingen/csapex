#ifndef TRAINER_NODE_H
#define TRAINER_NODE_H

/// COMPONENT
#include "analyzer_adapter.h"

/// FORWARD DECLARATION
class Trainer;

/**
 * @brief The TrainerAdapter class is a base class for Trainer adapters
 */
class TrainerAdapter : public AnalyzerAdapter
{
protected:
    /**
     * @brief TrainerAdapter
     * @param initial config
     * @param trainer The trainer to wrap
     * @throws if something went wrong
     */
    TrainerAdapter(Trainer& trainer);

public:
    /**
     * @brief ~TrainerAdapter
     */
    virtual ~TrainerAdapter();

protected:
    Trainer& trainer;
};

#endif // TRAINER_NODE_H
