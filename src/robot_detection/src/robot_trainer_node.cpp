/// PROJECT
#include <ros/ros_config.h>
#include <analyzer/trainer_online.h>
#include <ros/trainer_adapter_ros.h>
#include <db_strategy/factory.h>
#include <viz/analyzer_window.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv, "robot_trainer");

    ros::NodeHandle nh("~");

    Config cfg = RosConfig::importFromNodeHandle(nh);
    cfg["name"] = "Online Trainer";
    cfg.replaceInstance();

    TrainerOnline trainer;
    TrainerAdapterRos node(trainer);

    return node.run<AnalyzerWindow>(argc, argv);
}
