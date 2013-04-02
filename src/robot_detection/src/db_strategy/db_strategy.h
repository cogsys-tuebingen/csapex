#ifndef TRAINING_STRATEGY_H
#define TRAINING_STRATEGY_H

/// COMPONENT
#include "db_strategy_interface.h"

/**
 * @brief The DatabaseStrategy class is the interface between Analyzers and the Databases (Strategy Pattern)
 */
class DatabaseStrategy : public DatabaseStrategyInterface
{
public:
    /**
     * @brief DatabaseStrategy
     * @param db The Database to manage
     */
    DatabaseStrategy(Database* db);

    /**
     * @brief ~DatabaseStrategy
     */
    virtual ~DatabaseStrategy();

public:
    /**
     * @brief train Callback whenever there is a new Frame
     * @param frame the frame to handle
     */
    virtual void train(Frame::Ptr frame) = 0;

    /**
     * @brief detect find the best Pose for the current frame
     * @param frame the frame to detect poses on
     * @param out_roi Output: region of interest of the detected robot
     * @param score_out Output: score of the detected robot
     * @return the detected robot pose or Pose::NULL_POSE
     */
    virtual MatchablePose* detect(Frame::Ptr frame, cv::Rect& out_roi, double* score_out = NULL);

    /**
     * @brief getPoseByAngle Find the pose closest to a given angle
     * @param yaw target angle
     * @param index Output: index of the pose
     * @return the best pose
     */
    MatchablePose* getPoseByAngle(const double yaw, int* index = NULL) const;

    /**
     * @brief addValidationExample Add a frame to the validation examples
     * @param frame the frame to analyze
     */
    virtual void addValidationExample(Frame::Ptr frame) = 0;

    /**
     * @brief validate validate the database using positive and negative examples
     */
    virtual void validate() = 0;

    /**
     * @brief load loads the database at it's default path
     * @param cfg Output: the read config
     */
    virtual bool load();

    /**
     * @brief loadConfig Import a config from a saved file
     * @return true, iff a config could be read
     */
    virtual bool loadConfig();

    /**
     * @brief save saves the database to it's default path
     * @return true, iff the config could be saved
     */
    virtual bool save();

    /**
     * @brief clear clears the database
     */
    virtual void clear();

    /**
     * @brief dumpReference writes reference images to the specified path
     * @param path
     */
    virtual void dumpReference(const std::string& path);

    /**
     * @brief getDatabase Getter
     * @return the database
     */
    virtual Database* getDatabase() {
        return db;
    }

protected:
    virtual void addFrame(Frame::Ptr f, double scale = 1.0);
};

#endif // TRAINING_STRATEGY_H
