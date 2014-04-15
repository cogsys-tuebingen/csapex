#ifndef IMPORT_ROS_H
#define IMPORT_ROS_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <ros/ros.h>

namespace csapex {

class ImportRos : public Node
{
public:
    ImportRos();

    virtual void setup();
    virtual void process();

    virtual QIcon getIcon() const;

protected:
    void refresh();
    void update();
    void setTopic(const ros::master::TopicInfo& topic);

private:
    ConnectorOut* connector_;

    ros::Subscriber current_subscriber;

    std::string current_topic_;
};

}


#endif // IMPORT_ROS_H
