#ifndef ROBOT_TRAINER_H
#define ROBOT_TRAINER_H

/// COMPONENT
#include "analyzer.h"

/// FORWARD DECLARATION
class Matchable;

class Trainer : public Analyzer
{
public:
    /**
     * @brief The State enum
     */
    enum State {
        TRAINING_PAUSED, TRAINING, TRAINING_STOPPED
    };

public:
    /**
     * @brief Trainer
     * @throws if something went wrong
     */
    Trainer();

    /**
     * @brief ~Trainer
     */
    virtual ~Trainer();

public:
    /**
     * @brief analyzeCurrentFrame Analyze the current image and save robot poses
     */
    virtual void analyzeCurrentFrame(Frame::Ptr frame);

    /**
     * @brief addNegativeExample Add a frame to the negative examples
     * @return false, iff shutdown requested
     */
    bool addNegativeExample(Frame::Ptr frame);

    /**
     * @brief addValidationExample Add a frame to the validation examples
     * @return false, iff shutdown requested
     */
    bool addValidationExample(Frame::Ptr frame);

    /**
     * @brief validate validate the database using positive and negative examples
     */
    virtual void validate();

    /**
     * @brief tick Callback function: Is called on every iteration of the main loop
     * @param dt the time in seconds that has passed since the last call
     */
    virtual void tick(double dt);

    /**
     * @brief reset Reset the Trainer completely
     */
    virtual void reset();

    /**
     * @brief getState Accessor
     * @return the current state of the trainer
     */
    State getState() {
        return state;
    }

    /**
     * @brief startTraining Set training stae
     */
    virtual void startTraining();

    /**
     * @brief pauseTraining Set pause state
     */
    virtual void pauseTraining();

    /**
     * @brief stopTraining Set training-stopped state
     */
    virtual void stopTraining();

protected:
    /**
     * @brief training Is called in the training state
     * @param frame the current frame
     */
    virtual void training(Frame::Ptr frame);

    /**
     * @brief stopped Is called in the training_stopped state
     * @param frame the current frame
     */
    virtual void stopped(Frame::Ptr frame);

    /**
     * @brief paused Is called in the paused state
     * @param frame the current_frame
     */
    virtual void paused(Frame::Ptr frame);

    /**
     * @brief findMatchingReference Debug function: find best matching reference and display it
     */
    void findMatchingReference();

protected:
    State state;

    std::vector<std::pair<double, cv::Mat> > reference_imgs_;

    bool state_changed_;

    long frame;
};

#endif // ROBOT_TRAINER_H
