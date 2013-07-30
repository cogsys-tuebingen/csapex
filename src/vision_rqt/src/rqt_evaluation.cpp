/// HEADER
#include "rqt_evaluation.h"

/// PROJECT
#include <csapex/connection_type_manager.h>
#include <csapex/designer.h>
#include <vision_evaluator/messages_default.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QMenuBar>
#include <QBoxLayout>

PLUGINLIB_DECLARE_CLASS(vision_rqt, Vision, vision_rqt::Vision, rqt_gui_cpp::Plugin)

using namespace vision_rqt;

Vision::Vision()
{
}


void Vision::initPlugin(qt_gui_cpp::PluginContext& context)
{
    context_ = &context;

    eva_ = new csapex::EvaluationWindow;
    eva_->showMenu();

    context_->addWidget(eva_);
}

void Vision::shutdownPlugin()
{
}

void Vision::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    instance_settings.setValue("file", eva_->getDesigner()->getConfig().c_str());
}

void Vision::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    QString file = instance_settings.value("file").toString();
    if(!file.isEmpty()) {
        eva_->getDesigner()->setCurrentConfig(file.toUtf8().constData());
        eva_->getDesigner()->reload();
    }
}
