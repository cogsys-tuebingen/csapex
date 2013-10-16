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
    void stop();

    boost::shared_ptr<ros::NodeHandle> nh();
    void initHandle(bool try_only = false);

    bool isConnected();
    bool topicExists(const std::string& topic);

    void checkMasterConnection();
    bool waitForConnection();
    void refresh();

    void registerConnectionCallback(boost::function<void()>);

private:
    ROSHandler();

    boost::shared_ptr<ros::NodeHandle> nh_;
    boost::shared_ptr<ros::AsyncSpinner> spinner_;

    bool initialized_;
    QMutex has_connection_mutex;
    QFuture<bool> has_connection;

    std::vector<boost::function<void()> > callbacks_;
};

}

#endif // ROS_HANDLER_H
