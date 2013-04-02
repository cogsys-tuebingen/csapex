#ifndef DB_STRATEGY_DECORATOR_H
#define DB_STRATEGY_DECORATOR_H

/// COMPONENT
#include "db_strategy_interface.h"

class DatabaseStrategyDecorator : public DatabaseStrategyInterface
{
public:
    /**
     * @brief DatabaseStrategyDecorator
     * @param decorated
     */
    DatabaseStrategyDecorator(DatabaseStrategyInterface::Ptr decorated);

    /**
     * @brief ~DatabaseStrategyDecorator
     */
    ~DatabaseStrategyDecorator();

    /**
     * @brief handle_current_frame Callback whenever there is a new Frame
     * @param frame the frame to handle
     */
    void train(Frame::Ptr frame);

    /**
     * @brief detect find the best Pose for the current frame
     * @param frame the frame to detect poses on
     * @param out_roi Output: region of interest of the detected robot
     * @param score_out Output: score of the detected robot
     * @return the detected robot pose or Pose::NULL_POSE
     */
    MatchablePose* detect(Frame::Ptr frame, cv::Rect& out_roi, double* score_out = NULL) ;

    /**
     * @brief addValidationExample Add a frame to the validation examples
     * @param frame the frame to analyze
     */
    void addValidationExample(Frame::Ptr frame) ;

    /**
     * @brief getPoseByAngle Find the pose closest to a given angle
     * @param yaw target angle
     * @param index Output: index of the pose
     * @return the best pose
     */
    MatchablePose* getPoseByAngle(const double yaw, int* index = NULL) const;

    /**
     * @brief validate validate the database using positive and negative examples
     */
    void validate();

    /**
     * @brief load loads the database at it's default path
     */
    bool load();

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
    void clear();

    /**
     * @brief dumpReference writes reference images to the specified path
     * @param path
     */
    void dumpReference(const std::string& path);

    /**
     * @brief getDatabase Getter
     * @return the database
     */
    Database* getDatabase();

protected:
    DatabaseStrategyInterface::Ptr decorated;

private:
    long TAG;
};

#endif // DB_STRATEGY_DECORATOR_H
