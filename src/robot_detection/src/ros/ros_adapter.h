#ifndef ROS_ADAPTER_H
#define ROS_ADAPTER_H

/// PROJECT
#include <robot_detection/GlobalConfig.h>

/// SYSTEM
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <data/frame.h>

/**
 * @brief The RosAdapter class adapts an analyzer to the ros interface
 */
class RosAdapter
{
protected:
    /**
     * @brief RosAdapter Constructor
     */
    RosAdapter();

public:
    /**
     * @brief ~RosAdapter Destructor
     */
    virtual ~RosAdapter();

    /**
     * @brief shutdown Stop all execution
     */
    virtual void shutdown();

protected:
    /**
     * @brief spin loops the tick function
     */
    virtual void spin();

    /**
     * @brief import convert a ROS sensor_msgs:Image into a Frame
     * @param msg
     * @param roi optional region of interest
     * @return new Frame, responsibility for deleting given to caller
     */
    static Frame::Ptr convert(const sensor_msgs::ImageConstPtr& msg, cv::Rect roi = cv::Rect());

protected: // abstract
    virtual void tick(double dt) = 0;

protected:
    ros::NodeHandle nh;

private:
    boost::mutex shutdown_mutex;

    dynamic_reconfigure::Server<robot_detection::GlobalConfig> server;
    dynamic_reconfigure::Server<robot_detection::GlobalConfig>::CallbackType f;
};

#endif // ROS_ADAPTER_H
