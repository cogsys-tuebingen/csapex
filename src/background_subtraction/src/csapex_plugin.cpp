/// HEADER
#include "csapex_plugin.h"

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/manager/connection_type_manager.h>
#include <csapex/model/tag.h>

/// SYSTEM
#include <boost/bind.hpp>
#include <pluginlib/class_list_macros.h>
#include <QMetaType>

PLUGINLIB_EXPORT_CLASS(csapex::CSAPEXPlugin, csapex::CorePlugin)

using namespace csapex;

CSAPEXPlugin::CSAPEXPlugin()
{
}

void CSAPEXPlugin::init()
{
    Tag::createIfNotExists("BackgroundSubtraction");
}
