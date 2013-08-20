#ifndef ROS_HANDLER_H
#define ROS_HANDLER_H

/// SYSTEM
#include <boost/noncopyable.hpp>
#include <ros/ros.h>
#include <QFuture>

namespace csapex
{

class ROSHandler : public boost::noncopyable
{
public:
    static ROSHandler& instance();
    ~ROSHandler();

    boost::shared_ptr<ros::NodeHandle> nh();
    void initHandle(bool try_only = false);

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
