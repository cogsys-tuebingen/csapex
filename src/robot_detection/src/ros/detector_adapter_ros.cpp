/// HEADER
#include "detector_adapter_ros.h"

/// PROJECT
#include <analyzer/detector.h>
#include <data/frame_io.h>

/// SYSTEM
#include <geometry_msgs/PoseArray.h>

DetectorAdapterRos::DetectorAdapterRos(Detector& detector)
    : DetectorAdapter(detector)
{
    pose_pub = nh.advertise<geometry_msgs::PoseArray>
               ("poses", 5, true);
    img_sub = nh.subscribe<sensor_msgs::Image>
              ("/marlin/camera/image_raw", 1, boost::bind(&DetectorAdapterRos::imageCB, this, _1));
    img_alt_sub = nh.subscribe<sensor_msgs::Image>
                  ("/camera/image_raw", 1, boost::bind(&DetectorAdapterRos::imageCB, this, _1));
}

DetectorAdapterRos::~DetectorAdapterRos()
{
}

void DetectorAdapterRos::runHeadless()
{
    spin();
}

void DetectorAdapterRos::tick(double dt)
{
    detector.tick(dt);

    if(!poses_collected.empty()) {
        geometry_msgs::PoseArray msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/camera";

        for(std::vector<Pose>::iterator it = poses_collected.begin(); it != poses_collected.end(); ++it) {
            geometry_msgs::Pose pose;

            pose.position.x = it->position.x();
            pose.position.y = it->position.y();
            pose.position.z = it->position.z();

            pose.orientation.x = it->orientation.x();
            pose.orientation.y = it->orientation.y();
            pose.orientation.z = it->orientation.z();
            pose.orientation.w = it->orientation.w();

            msg.poses.push_back(pose);
        }

        poses_collected.clear();

        pose_pub.publish(msg);
    }
}

void DetectorAdapterRos::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
    detector.analyze(convert(msg));
}

void DetectorAdapterRos::poseDetected(const Pose& pose_camera)
{
    poses_collected.push_back(pose_camera);
}
