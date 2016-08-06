/// HEADER
#include <csapex/view/param/output_progress_param_adapter.h>

/// PROJECT
#include <csapex/view/utility/qwrapper.h>
#include <csapex/view/node/parameter_context_menu.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/utility/assert.h>
#include <csapex/utility/type.h>
#include <csapex/command/update_parameter.h>

/// SYSTEM
#include <QPointer>
#include <QBoxLayout>
#include <QProgressBar>
#include <iostream>

using namespace csapex;

OutputProgressParameterAdapter::OutputProgressParameterAdapter(param::OutputProgressParameter::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), op_p_(p)
{

}

QWidget* OutputProgressParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    QPointer<QProgressBar> bar = new QProgressBar;
    bar->setValue(op_p_->getProgress());
    bar->setMaximum(op_p_->getProgressMaximum());
    bar->setFormat(QString::fromStdString(op_p_->name()) + ": %p%");
    layout->addWidget(bar);

    // model change -> ui
    connectInGuiThread(op_p_->parameter_changed, [this, bar](param::Parameter*) {
        if(op_p_ && bar) {
            bar->setValue(op_p_->getProgress());
        }
    });

    // parameter scope changed -> update slider interval
    connectInGuiThread(op_p_->scope_changed, [this, bar](param::Parameter*) {
        if(op_p_ && bar) {
            bar->setMaximum(op_p_->getProgressMaximum());
        }
    });

    return bar;
}
