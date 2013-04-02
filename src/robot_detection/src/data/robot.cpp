#include "robot.h"

void Robot::add_pose(const Pose& pose)
{
    poses_.push_back(pose);
}

int Robot::pose_count()
{
    return poses_.size();
}

Pose Robot::pose(int index)
{
    return poses_[index];
}

void Robot::set_name(const std::string& name)
{
    name_ = name;
}
