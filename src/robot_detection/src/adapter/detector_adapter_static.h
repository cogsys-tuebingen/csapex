#ifndef DETECTOR_NODE_STATIC_H
#define DETECTOR_NODE_STATIC_H

/// COMPONENT
#include "detector_adapter.h"

/**
 * @brief The DetectorAdapterStatic class provides its adaptee input images from a directory
 */
class DetectorAdapterStatic : public DetectorAdapter
{
public:
    /**
     * @brief DetectorAdapterStatic
     * @param initial config
     * @param detector The detector to wrap
     * @throws if something went wrong
     */
    DetectorAdapterStatic(Detector& detector);

    /**
     * @brief ~DetectorAdapterStatic
     */
    virtual ~DetectorAdapterStatic();

    /**
     * @brief tick Callback function: Is called on every iteration of the main loop
     * @param dt the time in seconds that has passed since the last call
     */
    void tick(double dt);

    /**
     * @brief test_on Test the adapted detector an a set of images from a directory
     * @param path The path to the directory where the images are
     */
    void test_on(const std::string& path);

    /**
     * @brief test_on_callback Callback function. Is called when a new image has been tested
     * @param frames A vector of Matchables* that have been produced since the last call
     */
    void test_on_callback(std::vector<Frame::Ptr > &frames);

protected:
    virtual void poseDetected(const Pose& pose_camera);

private:
    bool has_path;
    std::string path_;
};

#endif // DETECTOR_NODE_STATIC_H
