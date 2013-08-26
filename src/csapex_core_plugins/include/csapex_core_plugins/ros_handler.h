#ifndef ROS_HANDLER_H
#define ROS_HANDLER_H

/// PROJECT
#include <utils_plugin/singleton.hpp>

/// SYSTEM
#include <ros/ros.h>
#include <QFuture>

namespace csapex
{

class ROSHandler : public Singleton<ROSHandler>
{
    friend class Singleton<ROSHandler>;

public:
    ~ROSHandler();

    boost::shared_ptr<ros::NodeHandle> nh();
    void initHandle(bool try_only = false);

    bool topicExists(const std::string& topic);

    void checkMasterConnection();
    bool waitForConnection();
    void refresh();

private:
    ROSHandler();

    boost::shared_ptr<ros::NodeHandle> nh_;
    boost::shared_ptr<ros::AsyncSpinner> spinner_;

    bool initialized_;
    QFuture<bool> has_connection;
};

}

#endif // ROS_HANDLER_H
