#ifndef APEX_ROS_INTERFACE_H
#define APEX_ROS_INTERFACE_H

/// PROJECT
#include <csapex/core/core_plugin.h>

/// SYSTEM
#include <ros/ros.h>
#include <std_msgs/String.h>

// TODO:
// * make this an apex core plugin
//  * advertise service to pause/enable apex
// * move ros specific stuff here

namespace csapex
{
class APEXRosInterface : public CorePlugin
{
public:
    APEXRosInterface();
    ~APEXRosInterface();
    void init(CsApexCore& core);

private:
    void command(const std_msgs::StringConstPtr &cmd);

private:
    ros::Subscriber command_sub_;
    CsApexCore* core_;
};
}
#endif // APEX_ROS_INTERFACE_H
