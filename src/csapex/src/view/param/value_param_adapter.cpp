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
#include <QInputDialog>
#include <QApplication>
#include <iostream>

using namespace csapex;

const int ValueParameterAdapter::DEFAULT_INT_STEP_SIZE = 1;
const double ValueParameterAdapter::DEFAULT_DOUBLE_STEP_SIZE = 0.001;

ValueParameterAdapter::ValueParameterAdapter(param::ValueParameter::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), value_p_(p)
{

}

QWidget* ValueParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
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
        connectInGuiThread(p_->parameter_changed, [this, txt](param::Parameter*) {
            if(p_ && txt) {
                txt->blockSignals(true);

                txt->setText(QString::fromStdString(p_->as<std::string>()));

                txt->blockSignals(false);
            }
        });

        return txt;

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
        connectInGuiThread(p_->parameter_changed, [this, box](param::Parameter*) {
            if(p_ && box) {
                box->blockSignals(true);

                box->setChecked(p_->as<bool>());

                box->blockSignals(false);
            }
        });

        return box;


    } else if(value_p_->is<double>()) {
        QPointer<QDoubleSpinBox> box = new QDoubleSpinBox;
        box->setDecimals(10);
        box->setSingleStep(value_p_->getDictionaryValue("step_size", DEFAULT_DOUBLE_STEP_SIZE));
        box->setMaximum(1e12);
        box->setMinimum(-1e12);
        box->setValue(value_p_->as<double>());        
        box->setKeyboardTracking(false);

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
        connectInGuiThread(p_->parameter_changed, [this, box](param::Parameter*) {
            if(p_ && box) {
                box->blockSignals(true);

                box->setValue(p_->as<double>());

                box->blockSignals(false);
            }
        });
        connectInGuiThread(p_->dictionary_entry_changed, [this, box](const std::string& key) {
            if(p_ && box) {
                if(key == "step_size") {
                    box->setSingleStep(p_->getDictionaryValue<double>(key));
                }
            }
        });

        return box;

    }  else if(value_p_->is<int>()) {
        QPointer<QSpinBox> box = new QSpinBox;
        box->setMaximum(std::numeric_limits<int>::max());
        box->setMinimum(std::numeric_limits<int>::min());
        box->setSingleStep(value_p_->getDictionaryValue("step_size", DEFAULT_INT_STEP_SIZE));
        box->setValue(value_p_->as<int>());
        box->setKeyboardTracking(false);

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
        connectInGuiThread(p_->parameter_changed, [this, box](param::Parameter*) {
            if(p_ && box) {
                box->blockSignals(true);

                box->setValue(p_->as<int>());

                box->blockSignals(false);
            }
        });
        connectInGuiThread(p_->dictionary_entry_changed, [this, box](const std::string& key) {
            if(p_ && box) {
                if(key == "step_size") {
                    box->setSingleStep(p_->getDictionaryValue<int>(key));
                }
            }
        });

        return box;

    } else {
        layout->addWidget(new QLabel((display_name + "'s type is not yet implemented (value: " + type2name(p_->type()) + ")").c_str()));
    }

    return nullptr;
}

void ValueParameterAdapter::setupContextMenu(ParameterContextMenu *context_handler)
{
    context_handler->addAction(new QAction("reset to default", context_handler), [this](){
        if(value_p_->is<std::string>()) {
            value_p_->set<std::string>(value_p_->def<std::string>());
        } else if(value_p_->is<bool>()) {
            value_p_->set<bool>(value_p_->def<bool>());
        } else if(value_p_->is<int>()) {
            value_p_->set<int>(value_p_->def<int>());
        } else if(value_p_->is<double>()) {
            value_p_->set<double>(value_p_->def<double>());
        }
    });

    if(value_p_->is<int>() || value_p_->is<double>()) {
        context_handler->addAction(new QAction("set step size", context_handler), [this](){
            if(value_p_->is<int>()) {
                int s = QInputDialog::getInt(QApplication::activeWindow(), "Step size", "Please enter the new step size",
                                             value_p_->getDictionaryValue("step_size", DEFAULT_INT_STEP_SIZE));
                value_p_->setDictionaryValue("step_size", s);
            } else if(value_p_->is<double>()) {
                double s = QInputDialog::getDouble(QApplication::activeWindow(), "Step size", "Please enter the new step size",
                                                   value_p_->getDictionaryValue("step_size", DEFAULT_DOUBLE_STEP_SIZE),
                                                   -1000., 1000.,
                                                   8);
                value_p_->setDictionaryValue("step_size", s);
            }
        });
    }
}
