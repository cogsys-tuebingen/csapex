/// PROJECT
#include <ros/ros_config.h>
#include <analyzer/detector.h>
#include <ros/detector_adapter_ros.h>
#include <db_strategy/factory.h>
#include <viz/analyzer_window.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv, "robot_detector");

    ros::NodeHandle nh("~");

    Config cfg = RosConfig::importFromNodeHandle(nh);
    cfg["name"] = "Online Detector";
    cfg.replaceInstance();

    Detector detector;
    DetectorAdapterRos node(detector);

    return node.run<AnalyzerWindow>(argc, argv);
}
