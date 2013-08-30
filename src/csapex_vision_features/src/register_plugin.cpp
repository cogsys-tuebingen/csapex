/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_vision_features/keypoint_message.h>
#include <csapex_vision_features/descriptor_message.h>

/// PROJECT
#include <csapex/connection_type_manager.h>
#include <csapex/tag.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::RegisterVisionFeaturePlugin, csapex::CorePlugin)

using namespace csapex;

RegisterVisionFeaturePlugin::RegisterVisionFeaturePlugin()
{
}

void RegisterVisionFeaturePlugin::init()
{
    Tag::createIfNotExists("Features");

    ConnectionTypeManager::registerMessage<connection_types::KeypointMessage>("std::vector<cv::KeyPoint>");
    ConnectionTypeManager::registerMessage<connection_types::DescriptorMessage>("cv::Mat");
}
