#ifndef RQT_EVALUATION_H
#define RQT_EVALUATION_H

/// PROJECT
#include <csapex/csapex_window.h>

/// SYSTEM
#include <rqt_gui_cpp/plugin.h>

namespace csapex_rqt {

class CsApex : public rqt_gui_cpp::Plugin
{
public:
    CsApex();
    virtual ~CsApex();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private:
    qt_gui_cpp::PluginContext* context_;

    csapex::CsApexCore core_;
    csapex::CsApexWindow* eva_;
};

}

#endif // RQT_EVALUATION_H
