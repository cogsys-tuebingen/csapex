/// HEADER
#include "ros_adapter.h"

/// PROJECT
#include <ros/ros_config.h>

/// SYSTEM
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

RosAdapter::RosAdapter()
    :  nh("~")
{
    f = boost::bind(&Config::replaceInstance, boost::bind(&RosConfig::import, _1, _2));
    server.setCallback(f);
}

RosAdapter::~RosAdapter()
{
    shutdown();
}

void RosAdapter::spin()
{
    ros::WallRate rate(30);

    ros::WallTime last(ros::WallTime::now());

    while(ros::ok() && !ros::isShuttingDown()) {
        ros::WallTime now = ros::WallTime::now();
        double dt = (now - last).toSec();
        last = now;

        {
            boost::mutex::scoped_lock(shutdown_mutex);
            tick(dt);
            ros::spinOnce();
        }
        rate.sleep();
    }
}

void RosAdapter::shutdown()
{
    boost::mutex::scoped_lock(shutdown_mutex);
    ros::shutdown();
}


Frame::Ptr RosAdapter::convert(const sensor_msgs::ImageConstPtr& msg, cv::Rect roi)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch(cv_bridge::Exception& e) {
        FATAL("cv_bridge exception: " << e.what());
        assert(false);
        return Frame::NULL_FRAME;
    }

    Frame::Ptr frame(new Frame(cv_ptr->image));
    if(roi.width > 0 && roi.height > 0) {
        frame->setRoi(roi);
    }

    return frame;
}
