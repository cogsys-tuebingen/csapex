/// HEADER
#include "rqt_evaluation.h"

/// PROJECT
#include <csapex/connection_type_manager.h>
#include <csapex/designer.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QMenuBar>
#include <QBoxLayout>

PLUGINLIB_DECLARE_CLASS(csapex_rqt, CsApex, csapex_rqt::CsApex, rqt_gui_cpp::Plugin)

using namespace csapex_rqt;

CsApex::CsApex()
{
}

CsApex::~CsApex()
{
    delete eva_;
}


void CsApex::initPlugin(qt_gui_cpp::PluginContext& context)
{
    context_ = &context;

    eva_ = new csapex::CsApexWindow();
    eva_->showMenu();

    context_->addWidget(eva_);
}

void CsApex::shutdownPlugin()
{
}

void CsApex::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    instance_settings.setValue("file", eva_->getConfig().c_str());
}

void CsApex::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    QString file = instance_settings.value("file").toString();
    if(!file.isEmpty()) {
        eva_->setCurrentConfig(file.toStdString());
        eva_->reload();
    }
}
