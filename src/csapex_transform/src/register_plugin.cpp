/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/manager/connection_type_manager.h>
#include <csapex/model/tag.h>
#include <csapex_core_plugins/ros_message_conversion.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::RegisterTransformPlugin, csapex::CorePlugin)

using namespace csapex;

RegisterTransformPlugin::RegisterTransformPlugin()
{
}

void RegisterTransformPlugin::init()
{
    Tag::createIfNotExists("Transform");
    Tag::createIfNotExists("Time");

    ConnectionTypeManager::registerMessage<connection_types::TransformMessage>();
}
