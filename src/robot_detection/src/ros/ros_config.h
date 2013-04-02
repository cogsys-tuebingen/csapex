#ifndef ROS_CONFIG_H
#define ROS_CONFIG_H

/// PROJECT
#include <config/config.h>
#include <robot_detection/GlobalConfig.h>

class RosConfig: public Config
{
public:
    /**
     * @brief import from a ros node
     * @param nh Nodehandle to initialize from
     * @return
     */
    static Config importFromNodeHandle(ros::NodeHandle& nh);

    /**
     * @brief import a ros generated config
     * @param cfg generated config to wrap
     * @return
     */
    static Config import(robot_detection::GlobalConfig& cfg, int level);

protected:
    virtual void make_defaults();
};
#endif // ROS_CONFIG_H
