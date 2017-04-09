/// HEADER
#include <csapex/view/param/trigger_param_adapter.h>

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
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QInputDialog>
#include <QApplication>
#include <iostream>

using namespace csapex;

TriggerParameterAdapter::TriggerParameterAdapter(param::TriggerParameter::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), value_p_(p)
{

}

QWidget* TriggerParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    QPointer<QPushButton> btn = new QPushButton(QString::fromStdString(p_->name()));

    QHBoxLayout* sub = new QHBoxLayout;
    sub->addWidget(btn);
    layout->addLayout(QtHelper::wrap(display_name, sub, context_handler));


    auto cb = [this](){
        command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>(AUUID(p_->getUUID()), "");
        executeCommand(update_parameter);
    };

    QObject::connect(btn.data(), &QPushButton::pressed, cb);

    return btn;
}

void TriggerParameterAdapter::setupContextMenu(ParameterContextMenu *context_handler)
{
}
