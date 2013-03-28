#ifndef IMAGE_PROVIDER_ROS_TOPIC_H
#define IMAGE_PROVIDER_ROS_TOPIC_H

/// COMPONENT
#include "image_provider.h"

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <QRadioButton>
#include <QFuture>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace vision_evaluator
{

class ImageProviderRosTopic : public ImageProvider
{
    Q_OBJECT

protected:
    ImageProviderRosTopic();
public:
    virtual ~ImageProviderRosTopic();

public:
    static boost::function<bool(ImageProvider*)> Identity;
private:
    static bool checkIdentity(ImageProvider*);

public:
    static ImageProvider* createInstance();
    static PluginPtr createMetaInstance();
    virtual PluginPtr metaInstance();

    bool hasNext();

public Q_SLOTS:
    void next();

    static void init(int argc, char** argv);
    virtual void init_gui(QToolBox* toolbox);

private:
    static void checkMasterConnection();
    void initHandle(bool try_only = false);
    void callback(const sensor_msgs::ImageConstPtr& msg);

public Q_SLOTS:
    void forceRefresh();
    void refresh();
    void changeTopic();

private:
    static QFuture<bool> has_connection;

private:
    ros::NodeHandle* nh;

    ros::Subscriber current_subscriber;

    QVBoxLayout* layout;
    std::vector<QRadioButton*> buttons;

    cv::Mat img;
};

} /// NAMESPACE

#endif // IMAGE_PROVIDER_ROS_TOPIC_H
