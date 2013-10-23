#ifndef REGISTER_APEX_PLUGIN_H
#define REGISTER_APEX_PLUGIN_H

/// SYSTEM
#include <pluginlib/class_list_macros.h>

#define CSAPEX_REGISTER_CLASS(name, parent) \
    PLUGINLIB_EXPORT_CLASS(name, parent)

#endif // REGISTER_PLUGIN_H
