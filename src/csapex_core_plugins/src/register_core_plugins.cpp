/// HEADER
#include "register_core_plugins.h"

/// COMPONENT
#include <csapex_core_plugins/string_message.h>

/// PROJECT
#include <csapex/connection_type_manager.h>
#include <csapex/tag.h>

/// SYSTEM
#include <boost/bind.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::RegisterCorePlugins, csapex::CorePlugin)

using namespace csapex;

RegisterCorePlugins::RegisterCorePlugins()
{
}

void RegisterCorePlugins::init()
{
    Tag::createIfNotExists("Buffer");
    Tag::createIfNotExists("General");
    Tag::createIfNotExists("Input");
    Tag::createIfNotExists("Output");
    Tag::createIfNotExists("RosIO");
    Tag::createIfNotExists("ConsoleIO");

    ConnectionTypeManager::registerMessage("std::string", boost::bind(&connection_types::StringMessage::make));
}
