/// HEADER
#include <csapex/view/param/angle_param_adapter.h>

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
#include <QDial>
#include <iostream>

using namespace csapex;

namespace
{

double normalizeAngle(double a)
{
    double r = a;
    while(r < -M_PI) r += 2*M_PI;
    while(r >= M_PI) r -= 2*M_PI;
    return r;
}

double angleToDial(double angle)
{
    return (normalizeAngle(angle) + M_PI) * 4.0 * 180.0 / M_PI;
}

double dialToAngle(double dial)
{
    return normalizeAngle(dial  / 4.0 / 180.0 * M_PI - M_PI);
}

}



AngleParameterAdapter::AngleParameterAdapter(param::AngleParameter::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), angle_p_(p)
{

}

QWidget *AngleParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    QLabel* label = new QLabel(angle_p_->name().c_str());

    label->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(label, &QLabel::customContextMenuRequested,
                     [=](const QPoint& point){ customContextMenuRequested(label, point); });

    layout->addWidget(label);

    QPointer<QDial> dial = new QDial;
    dial->setMinimum(0);
    dial->setMaximum(360.0 * 4);
    dial->setWrapping(true);
    dial->setValue(angleToDial(angle_p_->as<double>()));
    dial->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(dial.data(), &QDial::customContextMenuRequested,
                     [=](const QPoint& point){ customContextMenuRequested(dial, point); });
    layout->addWidget(dial);

    QPointer<QDoubleSpinBox> spin = new QDoubleSpinBox;
    spin->setMinimum(-M_PI);
    spin->setMaximum(M_PI);
    spin->setDecimals(5);
    spin->setSingleStep(0.001);
    spin->setValue(angle_p_->as<double>());
    spin->setKeyboardTracking(false);

    layout->addWidget(spin);

    // ui change -> model
    QObject::connect(dial.data(), &QDial::valueChanged, [this, dial, spin](int value){
        if(!angle_p_ || !dial  || !spin) {
            return;
        }

        double angle = dialToAngle(value);
        double min = angle_p_->min();
        double max = angle_p_->max();

        if(angle < min) {
            angle = min;
        } else if(angle > max) {
            angle = max;
        }

        set(angle);

        spin->blockSignals(true);
        spin->setValue(angle);
        spin->blockSignals(false);
    });

    QObject::connect(spin.data(), static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [this, dial, spin](double value){
        if(!angle_p_ || !dial  || !spin) {
            return;
        }

        double angle = value;
        double min = angle_p_->min();
        double max = angle_p_->max();

        if(angle < min) {
            angle = min;
        } else if(angle > max) {
            angle = max;
        }

        set(angle);

        dial->blockSignals(true);
        dial->setValue(angleToDial(angle_p_->as<double>()));
        dial->blockSignals(false);
    });

    // model change -> ui
    connectInGuiThread(angle_p_->parameter_changed, [this, dial, spin](param::Parameter*){
        if(!angle_p_ || !dial  || !spin) {
            return;
        }

        double angle = normalizeAngle(angle_p_->as<double>());
        double min = angle_p_->min();
        double max = angle_p_->max();

        if(angle < min) {
            angle = min;
        } else if(angle > max) {
            angle = max;
        }
        double val = angleToDial(angle);

        dial->blockSignals(true);
        spin->blockSignals(true);

        dial->setValue(val);
        spin->setValue(angle);

        dial->blockSignals(false);
        spin->blockSignals(false);
    });

    return label;
}

void AngleParameterAdapter::setupContextMenu(ParameterContextMenu *context_handler)
{
    context_handler->addAction(new QAction("set -π", context_handler), [this](){
        set(-M_PI);
    });
    context_handler->addAction(new QAction("set -π/2", context_handler), [this](){
        set(-M_PI_2);
    });
    context_handler->addAction(new QAction("set 0", context_handler), [this](){
        set(0.0);
    });
    context_handler->addAction(new QAction("set +π/2", context_handler), [this](){
        set(M_PI_2);
    });
    context_handler->addAction(new QAction("set +π", context_handler), [this](){
        set(M_PI-1e-9);
    });
}

void AngleParameterAdapter::set(double angle)
{
    command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>(AUUID(p_->getUUID()), angle);
    executeCommand(update_parameter);
}
