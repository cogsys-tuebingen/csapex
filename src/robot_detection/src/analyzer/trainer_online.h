#ifndef TRAINER_ONLINE_H
#define TRAINER_ONLINE_H

/// COMPONENT
#include "trainer.h"

/// FORWARD DECLARATION
class Detector;

/**
 * @brief The TrainerOnline class is used for the online training process and maintains a Detector
 */
class TrainerOnline : public Trainer
{
public:
    /**
     * @brief TrainerOnline
     * @throws if something went wrong
     */
    TrainerOnline();

    /**
     * @brief ~TrainerOnline()
     */
    ~TrainerOnline();

public:
    /**
     * @brief tick Callback function: Is called on every iteration of the main loop
     * @param dt the time in seconds that has passed since the last call
     */
    void tick(double dt);


protected:
    /**
     * @brief stopped Is called in the paused state
     * @param frame the current frame
     */
    void stopped(Frame::Ptr frame);

private:
    Detector* detector;
};

#endif // TRAINER_ONLINE_H
