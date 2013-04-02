#ifndef LASER_BACKGROUND_SUBTRACTOR_H
#define LASER_BACKGROUND_SUBTRACTOR_H

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/// FORWARD DECLARATION
class Context;

/**
 * @brief The LaserBackgroundSubtractor class estimates the distance of a movable object in a laserscan
 */
class LaserBackgroundSubtractor
{
public:
    /**
     * @brief LaserBackgroundSubtractor
     * @param nh ROS NodeHandle
     */
    LaserBackgroundSubtractor(const ros::NodeHandle& nh);

    /**
     * @brief process processes another laserscan
     * @param out_visualization Output: debug image
     * @param msg the scan as a ROS message type
     */
    void process(const sensor_msgs::LaserScanConstPtr& msg, cv::Mat& out_visualization);

    /**
     * @brief ready
     * @return true, iff the processor is ready to filter scans
     */
    bool ready();

    /**
     * @brief dist Accessor
     * @return the latest calculated distance
     */
    double dist();

private:
    void updateMap(const sensor_msgs::LaserScanConstPtr& msg);
    cv::Mat calcDist(const sensor_msgs::LaserScanConstPtr& msg);

private:
    double dim;
    int pixels;
    double res;

    cv::Mat map;

    int takes_max;
    int takes;

    int cx;
    int cy;

    double last_scan_dist;
};

#endif // LASER_BACKGROUND_SUBTRACTOR_H
