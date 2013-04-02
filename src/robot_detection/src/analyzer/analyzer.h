#ifndef ROBOT_ANALYZER_H
#define ROBOT_ANALYZER_H

/// PROJECT
#include <config/reconfigurable.h>
#include <data/frame.h>
#include <db_strategy/db_strategy.h>

/// SYSTEM
#include <boost/signals2.hpp>
#include <opencv2/core/core.hpp>

/// FORWARD DECLARATION
class Database;
class Extractor;
class Matcher;
class MatchScorer;

/**
 * @brief The Analyzer class is a base class for classes that analyze images.
 */
class Analyzer : public Reconfigurable
{
protected:
    /**
     * @brief Analyzer
     * @throws if something went wrong
     */
    Analyzer();

public:
    /**
     * @brief ~Analyzer
     */
    virtual ~Analyzer();

    /**
     * @brief tick Callback function: Is called on every iteration of the main loop
     * @param dt the time in seconds that has passed since the last call
     */
    virtual void tick(double dt);

    /**
     * @brief extractFeatures
     */
    void extractFeatures(Frame::Ptr frame, cv::Rect roi = cv::Rect(0,0,-1,-1));

    /**
     * @brief analyze Analyze a frame
     * @param frame the frame to analyze
     * @return false, iff shutdown requested
     */
    bool analyze(Frame::Ptr frame);

    /**
     * @brief finalizeDebugImage
     */
    virtual void finalizeDebugImage();

    /**
     * @brief getName Accessor for the name of this Analyzer
     * @return
     */
    std::string getName() {
        return config.name;
    }

    /**
     * @brief setDatabaseStrategy Setter
     * @param db_strat
     */
    void replaceDatabaseStrategy(DatabaseStrategyInterface::Ptr db_strat);

    /**
     * @brief getDatabaseStrategy Accessor
     * @return pointer to the used database strategy
     */
    DatabaseStrategyInterface::Ptr getDatabaseStrategy();

    /**
     * @brief getDatabase
     * @return pointer to the used database
     */
    Database* getDatabase();

protected: // abstract
    /**
     * @brief analyzeCurrentFrame Template function: Implemented by subclasses
     */
    virtual void analyzeCurrentFrame(Frame::Ptr frame) = 0;

public:
    /**
     * @brief tick_sig Signals that a tick has been received
     */
    boost::signals2::signal<void (double dt)> tick_sig;

    /**
     * @brief model_changed Signals that the model has been changed
     */
    boost::signals2::signal<void (Frame::Ptr frame)> frame_analyzed;

    /**
     * @brief trigger_model_changed signals model_changed with the current frame
     */
    void signal_frame_analyzed();

protected:
    void display_debug_image();

protected:
    DatabaseStrategyInterface::Ptr db_strategy;

    Frame::Ptr current_frame;
};

#endif // ROBOT_ANALYZER_H
