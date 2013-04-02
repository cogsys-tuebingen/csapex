#ifndef ROBOT_DETECTOR_H
#define ROBOT_DETECTOR_H

/// COMPONENT
#include "analyzer.h"

/// PROJECT
#include <data/pose.h>
#include <roi/roi.h>

/// SYSTEM
#include <boost/signals2.hpp>

/// FORWARD DECLARATION
class MatchablePose;
class Hypothesis;
class RoiManager;

/**
 * @brief The Detector class analyzes an image and publishes found robot poses
 */
class Detector : public Analyzer
{
public:
    /**
     * @brief Detector
     * @throws if something went wrong
     */
    Detector();

    /**
     * @brief ~Detector
     */
    virtual ~Detector();

    /**
     * @brief analyzeCurrentFrame Analyze the current image and find robot poses
     * @param frame the frame to analyze
     */
    void analyzeCurrentFrame(Frame::Ptr frame);

    /**
     * @brief analyzeRoi Analyze a part of the current image and find robot poses
     * @param frame the frame to analyze
     */
    void analyzeRoi(Frame::Ptr frame, const Roi& roi);

    /**
     * @brief tick Callback function: Is called on every iteration of the main loop
     * @param dt the time in seconds that has passed since the last call
     */
    void tick(double dt);

public:

    /**
     * @brief pose_detected Signals that a robot has been detected
     */
    boost::signals2::signal<void (const Pose&)> pose_detected;

private:
    void configChanged();
    void handle_pose(MatchablePose* best_match, Frame::Ptr frame, const Roi& roi);
    void drawDebugInfo(cv::Rect roi, MatchablePose* pose);

private:
    RoiManager* roi_manager;
};

#endif // ROBOT_DETECTOR_H
