/// HEADER
#include "rqt_evaluation.h"

/// PROJECT
#include <csapex/manager/connection_type_manager.h>
#include <csapex/view/designer.h>
#include <csapex/model/graph.h>
#include <csapex/core/graphio.h>
#include <csapex/view/design_board.h>
#include <csapex/view/designer.h>
#include <csapex/core/settings.h>
#include <csapex/manager/box_manager.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QMenuBar>
#include <QBoxLayout>

PLUGINLIB_DECLARE_CLASS(csapex_rqt, CsApex, csapex_rqt::CsApex, rqt_gui_cpp::Plugin)

using namespace csapex_rqt;
using namespace csapex;

CsApex::CsApex()
    : graph_(new Graph(settings_)), dispatcher_(new CommandDispatcher(graph_)), core_(settings_, graph_, dispatcher_), drag_io_(graph_.get(), dispatcher_)
{
    BoxManager::instance().settings_ = &settings_;
}

CsApex::~CsApex()
{
    delete eva_;
}


void CsApex::initPlugin(qt_gui_cpp::PluginContext& context)
{
    context_ = &context;

    board_ = new DesignBoard(graph_, dispatcher_, drag_io_);
    designer_ = new Designer(graph_, dispatcher_, board_);

    eva_ = new CsApexWindow(core_, dispatcher_, graph_, designer_);
    eva_->showMenu();

    context_->addWidget(eva_);
}

void CsApex::shutdownPlugin()
{
}

void CsApex::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    instance_settings.setValue("file", core_.getSettings().getConfig().c_str());
}

void CsApex::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    QString file = instance_settings.value("file").toString();
    if(!file.isEmpty()) {
        core_.getSettings().setCurrentConfig(file.toStdString());
        eva_->reload();
    }
}
