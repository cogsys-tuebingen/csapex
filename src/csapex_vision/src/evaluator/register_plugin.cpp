/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/connection_type_manager.h>
#include <csapex/tag.h>
#include <csapex_core_plugins/ros_message_conversion.h>
#include <csapex_core_plugins/ros_handler.h>
#include <csapex/connector_out.h>

/// SYSTEM
#include <boost/bind.hpp>
#include <pluginlib/class_list_macros.h>
#include <QMetaType>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

PLUGINLIB_EXPORT_CLASS(csapex::RegisterPlugin, csapex::CorePlugin)

using namespace csapex;

RegisterPlugin::RegisterPlugin()
{
}

struct Image2CvMat
{
    static void convert(const sensor_msgs::Image::ConstPtr &ros_msg, connection_types::CvMatMessage::Ptr& out) {
        try {
            cv_bridge::toCvShare(ros_msg, sensor_msgs::image_encodings::BGR8)->image.copyTo(out->value);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
};

void RegisterPlugin::init()
{
    Tag::createIfNotExists("Vision");
    Tag::createIfNotExists("Filter");
    Tag::createIfNotExists("Image Combiner");

    qRegisterMetaType<cv::Mat>("cv::Mat");

    ConnectionTypeManager::registerMessage("cv::Mat", boost::bind(&connection_types::CvMatMessage::make));

    RosMessageConversion::registerConversion("sensor_msgs/Image", RosMessageConversion::Convertor::Ptr(new RosMessageConversion::ConverterTemplate<sensor_msgs::Image, connection_types::CvMatMessage, Image2CvMat>));

    ConnectionType::default_.reset(new connection_types::CvMatMessage);
}
