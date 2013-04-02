#ifndef ROBOT_H
#define ROBOT_H

#include <vector>

#include "pose.h"

class Robot
{
public:
    void add_pose(const Pose& pose);
    void set_name(const std::string& name);

    int pose_count();
    Pose pose(int index);
    std::string name() {
        return name_;
    }

private:
    std::vector<Pose> poses_;
    std::string name_;
};

#endif // ROBOT_H
