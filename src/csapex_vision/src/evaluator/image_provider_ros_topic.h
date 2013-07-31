#ifndef IMAGE_PROVIDER_ROS_TOPIC_H
#define IMAGE_PROVIDER_ROS_TOPIC_H

/// COMPONENT
#include <csapex_vision/image_provider.h>

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <QRadioButton>
#include <QFuture>
#include <QToolBox>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace csapex
{

// @TODO this should be a "page plugin"

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
