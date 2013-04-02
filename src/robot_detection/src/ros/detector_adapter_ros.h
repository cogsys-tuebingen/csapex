#ifndef DETECTOR_NODE_ROS_H
#define DETECTOR_NODE_ROS_H

/// COMPONENT
#include "ros_adapter.h"

/// PROJECT
#include <adapter/detector_adapter.h>

/**
 * @brief The DetectorAdapterRos class uses a ROS context to provide its adaptee input image
 */
class DetectorAdapterRos : public DetectorAdapter, public RosAdapter
{
public:
    /**
     * @brief DetectorAdapterRos
     * @param initial config
     * @param detector The detector to use
     * @throws if something went wrong
     */
    DetectorAdapterRos(Detector& detector);

    /**
     * @brief ~DetectorAdapterRos
     */
    virtual ~DetectorAdapterRos();

    /**
     * @brief runHeadless Runs the main loop without a QtWindow
     */
    void runHeadless();

    /**
     * @brief tick Callback function: Is called on every iteration of the main loop
     * @param dt the time in seconds that has passed since the last call
     */
    void tick(double dt);

    /**
     * @brief imageCB Callback function. Is called when a new Image has arrived
     * @param msg The image in a ROS message format
     */
    void imageCB(const sensor_msgs::ImageConstPtr& msg);

protected:
    virtual void poseDetected(const Pose& pose_camera);

private:
    ros::Subscriber img_sub;
    ros::Subscriber img_alt_sub;

    ros::Publisher pose_pub;

    std::vector<Pose> poses_collected;
};

#endif // DETECTOR_NODE_ROS_H
