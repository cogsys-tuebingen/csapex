/// HEADER
#include <csapex/view/param/range_param_adapter.h>

/// PROJECT
#include <csapex/view/widgets/qint_slider.h>
#include <csapex/view/node/parameter_context_menu.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/utility/assert.h>
#include <csapex/utility/type.h>
#include <csapex/command/update_parameter.h>

/// SYSTEM
#include <QPointer>
#include <QBoxLayout>
#include <QLabel>
#include <QApplication>
#include <QInputDialog>
#include <QApplication>
#include <iostream>

using namespace csapex;

RangeParameterAdapter::RangeParameterAdapter(param::RangeParameter::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), range_p_(p),
      internal_layout(new QHBoxLayout)
{

}

namespace {
void setDecimals(QSpinBox*, int)
{
    // do nothing
}

void setDecimals(QDoubleSpinBox* box, double step)
{
    int decimals = 0;
    double v = step;
    while(v < 1.0) {
        v *= 10;
        ++decimals;
    }
    box->setDecimals(decimals);
}
}

QWidget* RangeParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    QLabel* label = new QLabel(QString::fromStdString(display_name));
    label->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(label, &QLabel::customContextMenuRequested,
                     [=](const QPoint& point){ customContextMenuRequested(label, point); });

    internal_layout->addWidget(label);

    if(range_p_->is<int>()) {
        genericSetup<int, QIntSlider, QWrapper::QSpinBoxExt>();
    } else if(range_p_->is<double>()) {
        genericSetup<double, QDoubleSlider, QWrapper::QDoubleSpinBoxExt>();
    } else {
        layout->addWidget(new QLabel((display_name + "'s type is not yet implemented (range: " + type2name(p_->type()) + ")").c_str()));
    }

    for(int i = 0; i < internal_layout->count(); ++i) {
        QWidget* child = internal_layout->itemAt(i)->widget();
        child->setProperty("parameter", QVariant::fromValue(static_cast<void*>(static_cast<csapex::param::Parameter*>(p_.get()))));
    }

    layout->addLayout(internal_layout);

    return label;
}

template <typename T, typename Slider, typename Spinbox>
void RangeParameterAdapter::genericSetup()
{
    T min = range_p_->min<T>();
    T max = range_p_->max<T>();
    T step = range_p_->step<T>();

    T def = range_p_->def<T>();

    if(((def - min) / step) * step != (def - min)) {
        std::cerr << "default " << def << " is not a multiple of minimum " << min << " with a step size of " << step << std::endl;
        def = min;
        std::cerr << "set default to " << def << std::endl;
    }

    if(((max - min) / step) * step != (max - min)) {
        std::cerr << "maximum " << max << " is not a multiple of minimum " << min << " with a step size of " << step << std::endl;
        max = ((max - min) / step) * step + min;
        std::cerr << "set maximum to " << max << std::endl;
    }

    QPointer<Slider> slider = new Slider(Qt::Horizontal, step);
    slider->setScaledMinimum(min);
    slider->setScaledMaximum(max);
    slider->setScaledValue(def);
    slider->setMinimumWidth(100);
    slider->setStepSize(step);

    QPointer<Spinbox> display = new Spinbox;
    display->setMinimum(min);
    display->setMaximum(max);
    display->setValue(def);
    display->setSingleStep(step);
    display->setKeyboardTracking(false);
    setDecimals(display.data(), step);

    internal_layout->addWidget(slider);
    internal_layout->addWidget(display);

    slider->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(slider.data(), &QLabel::customContextMenuRequested,
                     [=](const QPoint& point){ customContextMenuRequested(slider, point); });

    QObject::connect(slider.data(), &Slider::scaledValueChanged,
                     display.data(), &Spinbox::setValue);
    QObject::connect(slider.data(), &Slider::scaledRangeChanged,
                     display.data(), &Spinbox::setRange);
    QObject::connect(display.data(), static_cast<void(Spinbox::*)(T)>(&Spinbox::valueChanged),
                     slider.data(), &Slider::setScaledValue);


    slider->setScaledValue(p_->as<T>());

    // ui change -> model
    QObject::connect(slider.data(), &Slider::scaledValueChanged, [this, slider](T value){
        if(!p_ || !slider) {
            return;
        }

        command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>(AUUID(p_->getUUID()), value);
        executeCommand(update_parameter);
    });


    // model change -> ui
    connectInGuiThread(p_->parameter_changed, [this, slider, display](param::Parameter*){
        if(!p_ || !slider || !display) {
            return;
        }
        auto v = p_->as<T>();
        slider->blockSignals(true);
        display->blockSignals(true);

        slider->setScaledValue(v);
        display->setValue(v);

        display->blockSignals(false);
        slider->blockSignals(false);
    });
    connectInGuiThread(range_p_->step_changed, [this, slider, display](param::Parameter*){
        if(!range_p_ || !slider || !display) {
            return;
        }
        slider->blockSignals(true);
        display->blockSignals(true);

        auto step = range_p_->step<T>();

        slider->setStepSize(step);
        display->setSingleStep(step);

        display->blockSignals(false);
        slider->blockSignals(false);
    });


    // parameter scope changed -> update slider interval
    connectInGuiThread(p_->scope_changed, [this, slider, display](param::Parameter*) {
        if(!p_ || !slider || !display) {
            return;
        }
        slider->blockSignals(true);
        display->blockSignals(true);

        auto min = range_p_->min<T>();
        auto max = range_p_->max<T>();

        slider->setScaledRange(min, max);
        display->setMinimum(min);
        display->setMaximum(max);
        display->setValue(slider->scaledValue());

        display->blockSignals(false);
        slider->blockSignals(false);
    });
}

void RangeParameterAdapter::setupContextMenu(ParameterContextMenu *context_handler)
{
    context_handler->addAction(new QAction("reset to default", context_handler), [this](){
        if(range_p_->is<int>()) {
            range_p_->set(range_p_->def<int>());
        } else if(range_p_->is<double>()) {
            range_p_->set(range_p_->def<double>());
        }
    });

    context_handler->addAction(new QAction("set step size", context_handler), [this](){
        if(range_p_->is<int>()) {
            int s = QInputDialog::getInt(QApplication::activeWindow(), "Step size", "Please enter the new step size",
                                         range_p_->step<int>());
            range_p_->setStep(s);
        } else if(range_p_->is<double>()) {
            double s = QInputDialog::getDouble(QApplication::activeWindow(), "Step size", "Please enter the new step size",
                                               range_p_->step<double>(),
                                               -1000., 1000.,
                                               8);
            range_p_->setStep(s);
        }
    });
    context_handler->addAction(new QAction("set minimum", context_handler), [this](){
        if(range_p_->is<int>()) {
            int s = QInputDialog::getInt(QApplication::activeWindow(), "Minimum", "Please enter the new minimum value",
                                         range_p_->min<int>());
            range_p_->setMin(s);
        } else if(range_p_->is<double>()) {
            double s = QInputDialog::getDouble(QApplication::activeWindow(), "Minimum", "Please enter the new minimum value",
                                               range_p_->min<double>(),
                                               -10000000., 10000000.,
                                               8);
            range_p_->setMin(s);
        }
    });
    context_handler->addAction(new QAction("set maximum", context_handler), [this](){
        if(range_p_->is<int>()) {
            int s = QInputDialog::getInt(QApplication::activeWindow(), "Maximum", "Please enter the new maximum value",
                                         range_p_->max<int>());
            range_p_->setMax(s);
        } else if(range_p_->is<double>()) {
            double s = QInputDialog::getDouble(QApplication::activeWindow(), "Maximum", "Please enter the new maximum value",
                                               range_p_->max<double>(),
                                               -10000000., 10000000.,
                                               8);
            range_p_->setMax(s);
        }
    });
}
