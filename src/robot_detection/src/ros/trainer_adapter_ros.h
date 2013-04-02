#ifndef TRAINER_NODE_ROS_H
#define TRAINER_NODE_ROS_H

/// COMPONENT
#include "laser_background_subtractor.h"
#include "ros_adapter.h"

/// PROJECT
#include <adapter/trainer_adapter.h>
#include <config/config.h>

/// SYSTEM
#include <background_subtraction/MaskedImage.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

/// FORWARD DECLARATION
class Angle;

/**
 * @brief The TrainerAdapterRos class uses a ROS context to provide its adaptee input image
 */
class TrainerAdapterRos : public TrainerAdapter, public RosAdapter
{
public:
    /**
     * @brief TrainerAdapterRos
     * @param initial config
     * @param trainer The trainer to wrap
     * @throws if something went wrong
     */
    TrainerAdapterRos(Trainer& trainer);

    /**
     * @brief ~TrainerAdapterRos
     */
    virtual ~TrainerAdapterRos();

    /**
     * @brief runHeadless Runs the main loop without a QtWindow
     */
    virtual void runHeadless();

    /**
     * @brief tick Callback function: Is called on every iteration of the main loop
     * @param dt the time in seconds that has passed since the last call
     */
    void tick(double dt);

    /**
     * @brief image_callback Callback function. Is called when a new Image has arrived
     * @param msg The image in a ROS message format
     */
    void image_callback(const background_subtraction::MaskedImageConstPtr& msg);

    /**
     * @brief imu_callback Callback function. Is called when a new IMU measurement has arrived
     * @param msg The IMU message in a ROS message format
     */
    void imu_callback(const sensor_msgs::ImuConstPtr& msg);

    /**
     * @brief scan_callback Callback function. Is called when a new Laserscan measurement has arrived
     * @param msg The scan in a ROS message format
     */
    void scan_callback(const sensor_msgs::LaserScanConstPtr& msg);

private:
    virtual void get_transformation(const ros::Time& time, const Angle& yaw, double& dist);

private:
    ros::Subscriber img_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber scan_sub;

    ros::Time last_stamp;

    LaserBackgroundSubtractor laser_bg;

    bool has_imu;
    tf::Quaternion last_imu_orientation;

    tf::TransformListener tfl;

    bool use_tf;
    bool use_scan;
    bool use_imu;
    bool use_distance;
};

#endif // TRAINER_NODE_ROS_H
