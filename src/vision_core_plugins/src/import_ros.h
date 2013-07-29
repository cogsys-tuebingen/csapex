#ifndef IMPORT_ROS_H
#define IMPORT_ROS_H

/// PROJECT
#include <designer/boxed_object.h>

/// SYSTEM
#include <ros/ros.h>
#include <QFuture>

namespace vision_evaluator {

class ImportRos : public BoxedObject
{
    Q_OBJECT
public:
    ImportRos();

    virtual void fill(QBoxLayout* layout);
    virtual void updateDynamicGui(QBoxLayout *layout);
    virtual void messageArrived(ConnectorIn* source);
    virtual void tick();

public Q_SLOTS:
    void checkMasterConnection();
    void refresh();
    void initHandle(bool try_only = false);
    void changeTopic(const QString &topic);

private:
    void setTopic(const ros::master::TopicInfo& topic);

    template <class T>
    void callback(const typename T::ConstPtr& msg);

private:
    ConnectorOut* connector_;

    QVBoxLayout* dynamic_layout;

    boost::shared_ptr<ros::NodeHandle> nh;

    ros::Subscriber current_subscriber;

    bool initialized_;
    QFuture<bool> has_connection;

    std::string topic_;
};

}


#endif // IMPORT_ROS_H
