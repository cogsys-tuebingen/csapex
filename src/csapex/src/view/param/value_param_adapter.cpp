/// HEADER
#include <csapex/view/param/value_param_adapter.h>

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
#include <iostream>

using namespace csapex;

ValueParameterAdapter::ValueParameterAdapter(param::ValueParameter::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), value_p_(p)
{

}

void ValueParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    if(value_p_->is<std::string>()) {
        QPointer<QLineEdit> txt = new QLineEdit;
        txt->setText(value_p_->as<std::string>().c_str());
        QPointer<QPushButton> send = new QPushButton("set");

        QHBoxLayout* sub = new QHBoxLayout;

        sub->addWidget(txt);
        sub->addWidget(send);

        layout->addLayout(QtHelper::wrap(display_name, sub, context_handler, p_.get()));

        // ui change -> model
        auto cb = [this, txt](){
            command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>(AUUID(p_->getUUID()), txt->text().toStdString());
            executeCommand(update_parameter);
        };

        QObject::connect(txt.data(), &QLineEdit::returnPressed, cb);
        QObject::connect(send.data(), &QPushButton::pressed, cb);

        // model change -> ui
        connectInGuiThread(p_->parameter_changed, [this, txt]() {
            if(p_ && txt) {
                txt->blockSignals(true);

                txt->setText(QString::fromStdString(p_->as<std::string>()));

                txt->blockSignals(false);
            }
        });

    } else if(value_p_->is<bool>()) {
        QPointer<QCheckBox> box = new QCheckBox;
        box->setChecked(value_p_->as<bool>());

        layout->addLayout(QtHelper::wrap(display_name, box, context_handler, p_.get()));

        // ui change -> model
        QObject::connect(box.data(), &QCheckBox::toggled, [this, box](){
            if(!p_ || !box) {
                return;
            }
            command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>(AUUID(p_->getUUID()), box->isChecked());
            executeCommand(update_parameter);
        });

        // model change -> ui
        connectInGuiThread(p_->parameter_changed, [this, box]() {
            if(p_ && box) {
                box->blockSignals(true);

                box->setChecked(p_->as<bool>());

                box->blockSignals(false);
            }
        });


    } else if(value_p_->is<double>()) {
        QPointer<QDoubleSpinBox> box = new QDoubleSpinBox;
        box->setDecimals(10);
        box->setMaximum(1e12);
        box->setMinimum(-1e12);
        box->setValue(value_p_->as<double>());

        layout->addLayout(QtHelper::wrap(display_name, box, context_handler, p_.get()));

        // ui change -> model
        QObject::connect(box.data(), static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [this, box](double value){
            if(!p_ || !box) {
                return;
            }
            command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>(AUUID(p_->getUUID()), value);
            executeCommand(update_parameter);
        });


        // model change -> ui
        connectInGuiThread(p_->parameter_changed, [this, box]() {
            if(p_ && box) {
                box->blockSignals(true);

                box->setValue(p_->as<double>());

                box->blockSignals(false);
            }
        });

    }  else if(value_p_->is<int>()) {
        QPointer<QSpinBox> box = new QSpinBox;
        box->setMaximum(std::numeric_limits<int>::max());
        box->setMinimum(std::numeric_limits<int>::min());
        box->setValue(value_p_->as<int>());

        layout->addLayout(QtHelper::wrap(display_name, box, context_handler,  p_.get()));

        // ui change -> model
        QObject::connect(box.data(), static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), [this, box](int value){
            if(!p_ || !box) {
                return;
            }
            command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>(AUUID(p_->getUUID()), value);
            executeCommand(update_parameter);
        });

        // model change -> ui
        connectInGuiThread(p_->parameter_changed, [this, box]() {
            if(p_ && box) {
                box->blockSignals(true);

                box->setValue(p_->as<int>());

                box->blockSignals(false);
            }
        });

    } else {
        layout->addWidget(new QLabel((display_name + "'s type is not yet implemented (value: " + type2name(p_->type()) + ")").c_str()));
    }
}
