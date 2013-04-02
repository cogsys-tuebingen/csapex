#ifndef TRACKER_ADAPTER_ROS_H
#define TRACKER_ADAPTER_ROS_H

/// PROJECT
#include <tracker/tracker.h>

/// SYSTEM
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

/**
 * @brief The TrackerAdapterRos class adapts the Tracker class to the ROS interface
 */
class TrackerAdapterRos
{
public:
    /**
     * @brief TrackerAdapterRos
     * @param tracker
     */
    TrackerAdapterRos(Tracker& tracker);

    /**
     * @brief run Runs the main loop
     * @param argc Parameters
     * @param argv Parameters
     * @return exit code
     */
    int run(int argc, char** argv);

    /**
     * @brief measurementCallback is called whenever a new pose has been detected
     * @param msg array of detected poses
     */
    void measurementCallback(const geometry_msgs::PoseArrayConstPtr& msg);

private:
    void addHypothesisToMarkerArray(visualization_msgs::MarkerArray& markers, geometry_msgs::PoseArray& poses, const Hypothesis& hypo, bool is_valid);
    void publishHypotheses();

private:
    ros::NodeHandle nh;
    tf::TransformListener tfl;

    Tracker& tracker;

    ros::Subscriber measurements_sub;

    std::string tracking_frame_id_;

    ros::WallRate publish_rate_;
    ros::WallTime last_publish_;

    ros::Publisher hypo_pub;
    ros::Publisher cov_pub;

    int nextMarkerId;
};

#endif // TRACKER_ADAPTER_ROS_H
