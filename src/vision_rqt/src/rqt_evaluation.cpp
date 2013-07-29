/// HEADER
#include "rqt_evaluation.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QStringList>

PLUGINLIB_DECLARE_CLASS(vision_rqt, Vision, vision_rqt::Vision, rqt_gui_cpp::Plugin)

using namespace vision_rqt;

Vision::Vision()
{
}


void Vision::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();

}

void Vision::shutdownPlugin()
{
  // TODO unregister all publishers here
}

void Vision::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void Vision::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}
