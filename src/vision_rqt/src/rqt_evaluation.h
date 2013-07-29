#ifndef RQT_EVALUATION_H
#define RQT_EVALUATION_H

/// SYSTEM
#include <rqt_gui_cpp/plugin.h>

namespace vision_rqt {

class Vision : public rqt_gui_cpp::Plugin
{
public:
    Vision();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
};

}

#endif // RQT_EVALUATION_H
