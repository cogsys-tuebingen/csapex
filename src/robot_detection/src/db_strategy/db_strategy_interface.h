#ifndef DB_STRATEGY_INTERFACE_H
#define DB_STRATEGY_INTERFACE_H

/// PROJECT
#include <config/reconfigurable.h>
#include <data/frame.h>

/// SYSTEM
#include <boost/signals2.hpp>
#include <opencv2/core/core.hpp>

/// FORWARD DECLARATION
class Context;
class Database;
class Extractor;
class Matchable;
class Matcher;
class MatchScorer;
class MatchablePose;

/**
 * @brief The DatabaseStrategyInterface class
 */
class DatabaseStrategyInterface : public Reconfigurable
{
public:
    typedef boost::shared_ptr<DatabaseStrategyInterface> Ptr;

protected:
    /**
     * @brief DatabaseStrategyInterface
     */
    DatabaseStrategyInterface(Database* db);

    /**
     * @brief DatabaseStrategyInterface
     * @param copy
     */
    DatabaseStrategyInterface(DatabaseStrategyInterface& copy);

public:
    /**
     * @brief ~DatabaseStrategyInterface
     */
    virtual ~DatabaseStrategyInterface();

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
    virtual MatchablePose* detect(Frame::Ptr frame, cv::Rect& out_roi, double* score_out = NULL) = 0;

    /**
     * @brief addValidationExample Add a frame to the validation examples
     * @param frame the frame to analyze
     */
    virtual void addValidationExample(Frame::Ptr frame) = 0;

    /**
     * @brief getPoseByAngle Find the pose closest to a given angle
     * @param yaw target angle
     * @param index Output: index of the pose
     * @return the best pose
     */
    virtual MatchablePose* getPoseByAngle(const double yaw, int* index = NULL) const = 0;

    /**
     * @brief validate validate the database using positive and negative examples
     */
    virtual void validate() = 0;

    /**
     * @brief load loads the database at it's default path
     */
    virtual bool load() = 0;

    /**
     * @brief loadConfig Import a config from a saved file
     * @return true, iff a config could be read
     */
    virtual bool loadConfig() = 0;

    /**
     * @brief save saves the database to it's default path
     * @return true, iff the config could be saved
     */
    virtual bool save() = 0;

    /**
     * @brief clear clears the database
     */
    virtual void clear() = 0;

    /**
     * @brief dumpReference writes reference images to the specified path
     * @param path
     */
    virtual void dumpReference(const std::string& path) = 0;

    /**
     * @brief getDatabase Getter
     * @return the database
     */
    virtual Database* getDatabase() = 0;

public:
//    boost::signals2::signal<void(const Config&)> reconfigure;

protected:
    Database* db;
};

#endif // DB_STRATEGY_INTERFACE_H
